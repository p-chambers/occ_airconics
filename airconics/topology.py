# -*- coding: utf-8 -*-
"""
Classses and functions used to define arbitrary aircraft topologies in
Airconics, inspired by the GPLearn API

Created on Fri Apr 15 12:24:32 2016

@author: pchambers
"""
from .base import AirconicsCollection
from .primitives import Airfoil
from .fuselage_oml import Fuselage
from .liftingsurface import LiftingSurface
from .engine import Engine
from . import AirCONICStools as act
from .examples.wing_example_transonic_airliner import *
from .examples.tailplane_example_transonic_airliner import *
from .examples.straight_wing import *
from .examples.tapered_wing import *
from OCC.gp import gp_Ax2, gp_Ax1, gp_Dir, gp_Pnt

import types
import functools
from functools import partial
import itertools
import pydot
import re
from collections import OrderedDict
import numpy as np
from deap import algorithms
from deap import base
from deap import creator
from deap import tools
from deap import gp
import json
import sys
import os
from operator import attrgetter


import logging
log = logging.getLogger(__name__)

# This dictionary will be used for topology tree formatting
FUNCTIONS = {'E': Fuselage,         # E = Enclosure
             'L': LiftingSurface,   # L = Lifting Surface
             'P': Engine,           # P = Propulsion
             '|': gp_Ax2,           # M = Mirror Plane
             '': None}

# Reversed dictionary for manually adding shapes, i.e. converting
#  a class instance to a string
FUNCTIONS_INV = {func: name for name, func in FUNCTIONS.items()}

# The shapes of nodes in the exported graph from Topo class:
NODE_PROPERTIES = {'fuselage': {'shape': 'ellipse', 'fillcolor': '#136ed4', 'fontcolor': "white"},
                   'liftingsurface': {'shape': 'box', 'fillcolor': '#136ed4', 'fontcolor': "white"},
                   'engine': {'shape': 'hexagon', 'fillcolor': '#136ed4', 'fontcolor': "white"},
                   'mirror': {'shape': 'box', 'fillcolor': '#136ed4', 'fontcolor': "white"},
                   'number': {'shape': 'ellipse'},
                   'function': {'shape': 'ellipse'},
                   'None': {'shape': 'point'},
                   }

LSURF_FUNCTIONS = {'AirlinerWing':
   OrderedDict([('SweepFunct', mySweepAngleFunctionAirliner),
                ('DihedralFunct', myDihedralFunctionAirliner),
                ('ChordFunct', myChordFunctionAirliner),
                ('TwistFunct', myTwistFunctionAirliner),
                ('AirfoilFunct', myAirfoilFunctionAirliner)
                ]),
   'AirlinerTP':
   OrderedDict([('SweepFunct', mySweepAngleFunctionTP),
                ('DihedralFunct', myDihedralFunctionTP),
                ('ChordFunct', myChordFunctionTP),
                ('TwistFunct', myTwistFunctionTP),
                ('AirfoilFunct', myAirfoilFunctionTP)
                ]),
   'AirlinerFin':
   OrderedDict([('SweepFunct', mySweepAngleFunctionFin),
                ('DihedralFunct', myDihedralFunctionFin),
                ('ChordFunct', myChordFunctionFin),
                ('TwistFunct', myTwistFunctionFin),
                ('AirfoilFunct', myAirfoilFunctionFin)
                ]),
   'StraightWing':
   OrderedDict([('SweepFunct', SimpleSweepFunction),
                ('DihedralFunct', SimpleDihedralFunction),
                ('ChordFunct', SimpleChordFunction),
                ('TwistFunct', SimpleTwistFunction),
                ('AirfoilFunct', SimpleAirfoilFunction)
                ]),
   'TaperedWing':
   OrderedDict([('SweepFunct', TaperedWingSweepFunction),
                ('DihedralFunct', TaperedWingDihedralFunction),
                ('ChordFunct', TaperedWingChordFunction),
                ('TwistFunct', TaperedWingTwistFunction),
                ('AirfoilFunct', TaperedWingAirfoilFunction)
                ])
        }


# Use a wrapper to convert boxN, sphereN etc. to another function that
# returns a callable
def wrap_shapeN(shapeN):
    @functools.wraps(shapeN)
    def wrapped_shapeN(*args, **kwargs):
        return partial(shapeN, *args)
    return wrapped_shapeN


def wrap_fitnessfunct(fitness_funct):
    @functools.wraps(fitness_funct)
    def wrapped_fitnessfunct(self, topology):
        return fitness_funct(topology)
    return wrapped_fitnessfunct


def default_fitness(topology):
    """The default fitness function used by the Topology class

    Parameters
    ----------
    topology - Topology object
        The topology containing the geometry and component hierarchy
        information for which to calculate the 'current' fitness

    Notes
    -----
    Until I come with a better fitness function, this fitness function simply
    tries to maximise the volume of the bounding box
    """
    xmin, ymin, zmin, xmax, ymax, zmax = topology.Extents()
    return (xmax - xmin) * (ymax - ymin) * (zmax - zmin)


def create_diffed_airliner_fitness():
    """
    Produces a fitness function for checking the similarity of an input
    airconics topology with a reference: the airliner example.

    Uses difflib's gestalt pattern matching algorithms.

    Returns
    -------~
    diffed_fitness : function
        A function which takes an airconics topology as its input

    Notes
    -----
    * This function is used so that the reference geometry/layout needs to
    be created once only.
    
    * Eventually this function should use some information from the
    geometry model, e.g., total wing area, as part of the objective function.
    """
    # Create the reference list of primitive strings
    topo_tools = Topology_GPTools(MaxAttachments=4)
    fname = os.path.join(os.path.dirname(__file__),
                         'resources/configuration_app/presets/airliner.json')
    topo = topo_tools.from_file(fname)
    
    # Obtain a list of all component primitives in the tree
    airliner_json = topo.to_json()

    airliner_components = [node['primitive'] for node in airliner_json]

    def diffed_airliner_fitness(topology):
        """Currently just uses difflibs gestalt pattern matcher. Eventually,
        this should use data from the inputs of the matched components 
        """
        import difflib

        tree_json = topology.to_json()
        tree_components = [node['primitive'] for node in tree_json]

        sm = difflib.SequenceMatcher(None, tree_components, airliner_components)
        
        # Will give something in order 0-10, 0 being exactly the same
        diff = (1-sm.ratio()) * 10

        for match in sm.get_matching_blocks()[:-1]:

            for i in range(match.size):
                argsa = tree_json[match.a + i]['args']
                diff_vector = np.zeros(len(argsa))
                if len(argsa) > 0:
                    # This node is not a mirror, and inputs may have some difference
                    argsb = airliner_json[match.b + i]['args']

                    # Try and get the shape difference only:
                    try:
                        # if this works, it's a wing
                        diff += (1 if argsa['Type'] != argsb['Type'] else 0)
                    except KeyError:
                        # If not, it's a fuselage
                        pass
                    
                    for j, (key, val) in enumerate(argsb.items()):
                        try: 
                            diff_vector[j] = abs(val - argsa[key])
                        except TypeError:
                            # This is probably the string parameter (eg wing type)
                            pass
                    diff += np.linalg.norm(diff_vector) / np.linalg.norm(np.ones_like(diff_vector))   # Will give something in the order 1

        return diff
    return diffed_airliner_fitness



import random
from inspect import isclass


def generate_topology(pset, min_, max_, type_=None):
    """Generate a Tree as a list of list. The tree is build
    from the root to the leaves, and it stop growing when the
    condition is fulfilled.
    :param pset: Primitive set from which primitives are selected.
    :param min_: Minimum height of the produced trees. INCLUDES LOWEST LEVEL OF TERMINALS,
                 i.e., fuselage0(args) has a tree of height 2
    :param max_: Maximum Height of the produced trees.
    :param condition: The condition is a function that takes two arguments,
                      the height of the tree to build and the current
                      depth in the tree.
    :param type_: The type that should return the tree when called, when
                  :obj:`None` (default) the type of :pset: (pset.ret)
                  is assumed.
    :returns: A grown tree with leaves at possibly different depths
              dependending on the condition function.
    """
    if type_ is None:
        type_ = pset.ret
    expr = []
    height = random.randint(min_, max_)
    last_level = height - 1
    stack = [(0, type_)]
    while len(stack) != 0:
        depth, type_ = stack.pop()
        if depth == height:
            try:
                term = random.choice(pset.terminals[type_])
            except IndexError:
                _, _, traceback = sys.exc_info()
                raise IndexError("The topology.generate function tried to add "\
                                  "a terminal of type '%s', but there is "\
                                  "none available." % (type_,), traceback)
            if isclass(term):
                term = term()
            expr.append(term)
        else:
            try:
                if depth == last_level:
                    prim = pset.mapping[random.choice(pset.leaf_primitives[type_])]
                else:
                    prim = random.choice(pset.primitives[type_])
            except IndexError:
                try:
                    term = random.choice(pset.terminals[type_])
                except IndexError:
                    _, _, traceback = sys.exc_info()
                    raise IndexError(
                        """The topology.generate function tried to
                        add a terminal of type '%s', but there is none
                        available.""" % (type_,), traceback)
                if isclass(term):
                    term = term()
                expr.append(term)
            else:
                expr.append(prim)
                for arg in reversed(prim.args):
                    stack.append((depth + 1, arg))
    return expr


def eaSimple_logbest(population, toolbox, cxpb, mutpb, ngen, stats=None,
             halloffame=None, verbose=__debug__):
    """This algorithm is directly copied and edited from
    deap.algorithms.eaSimple, and reproduce the simplest evolutionary algorithm
    as presented in chapter 7 of [Back2000]_.

    Edits in this algorithm allow that the best individual of each generation
    is logged and returned.

    :param population: A list of individuals.
    :param toolbox: A :class:`~deap.base.Toolbox` that contains the evolution
                    operators.
    :param cxpb: The probability of mating two individuals.
    :param mutpb: The probability of mutating an individual.
    :param ngen: The number of generation.
    :param stats: A :class:`~deap.tools.Statistics` object that is updated
                  inplace, optional.
    :param halloffame: A :class:`~deap.tools.HallOfFame` object that will
                       contain the best individuals, optional.
    :param verbose: Whether or not to log the statistics.
    :returns: The final population
    :returns: A class:`~deap.tools.Logbook` with the statistics of the
              evolution
    :returns: The best individuals at each generation

    The algorithm takes in a population and evolves it in place using the
    :meth:`varAnd` method. It returns the optimized population and a
    :class:`~deap.tools.Logbook` with the statistics of the evolution. The
    logbook will contain the generation number, the number of evalutions for
    each generation and the statistics if a :class:`~deap.tools.Statistics` is
    given as argument. The *cxpb* and *mutpb* arguments are passed to the
    :func:`varAnd` function. The pseudocode goes as follow ::
        evaluate(population)
        for g in range(ngen):
            population = select(population, len(population))
            offspring = varAnd(population, toolbox, cxpb, mutpb)
            evaluate(offspring)
            population = offspring
    As stated in the pseudocode above, the algorithm goes as follow. First, it
    evaluates the individuals with an invalid fitness. Second, it enters the
    generational loop where the selection procedure is applied to entirely
    replace the parental population. The 1:1 replacement ratio of this
    algorithm **requires** the selection procedure to be stochastic and to
    select multiple times the same individual, for example,
    :func:`~deap.tools.selTournament` and :func:`~deap.tools.selRoulette`.
    Third, it applies the :func:`varAnd` function to produce the next
    generation population. Fourth, it evaluates the new individuals and
    compute the statistics on this population. Finally, when *ngen*
    generations are done, the algorithm returns a tuple with the final
    population and a :class:`~deap.tools.Logbook` of the evolution.
    .. note::
        Using a non-stochastic selection method will result in no selection as
        the operator selects *n* individuals from a pool of *n*.
    This function expects the :meth:`toolbox.mate`, :meth:`toolbox.mutate`,
    :meth:`toolbox.select` and :meth:`toolbox.evaluate` aliases to be
    registered in the toolbox.
    .. [Back2000] Back, Fogel and Michalewicz, "Evolutionary Computation 1 :
       Basic Algorithms and Operators", 2000.
    """
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals'] + (stats.fields if stats else [])

    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    if halloffame is not None:
        halloffame.update(population)

    record = stats.compile(population) if stats else {}
    logbook.record(gen=0, nevals=len(invalid_ind), **record)
    if verbose:
        print(logbook.stream)

    gen_best = [toolbox.clone(max(population, key=attrgetter("fitness")))]

    # Begin the generational process
    for gen in range(1, ngen + 1):
        # Select the next generation individuals
        offspring = toolbox.select(population, len(population))

        # Vary the pool of individuals
        offspring = algorithms.varAnd(offspring, toolbox, cxpb, mutpb)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # Update the hall of fame with the generated individuals
        if halloffame is not None:
            halloffame.update(offspring)

        # Replace the current population by the offspring
        population[:] = offspring

        gen_best.append(toolbox.clone(max(population, key=attrgetter("fitness"))))

        # Append the current generation statistics to the logbook
        record = stats.compile(population) if stats else {}
        logbook.record(gen=gen, nevals=len(invalid_ind), **record)
        if verbose:
            print(logbook.stream)

    return population, logbook, gen_best


class TreeNode(object):
    def __init__(self, part, name, arity):
        """Basic type to define node elements in the topology tree. To be used
        by Topology class.

        Parameters
        ----------
        part - Airconics type Fuselage, LiftingSurface or Engine
            The type to convert

        name - string
            The name of the part (e.g. 'Wing' or 'Fin')

        arity - int
            the number of descendants

        Attributes
        ----------
        arity - int
            arity (number of descendants) of this node.

        name - string
            Name of the part

        func - string
            Indicates the type of node i.e.
        """
        self.name = name
        self.arity = arity

        if type(part) not in FUNCTIONS.values():
            raise TypeError("Not a recognised part type: {}. Should be {}"
                            .format(type(part), FUNCTIONS.values()))
        else:
            func_str = FUNCTIONS_INV[type(part)]
        self.func = func_str

    def __str__(self):
        output = '({}, {}, {})'.format(self.name, self.func, self.arity)
        return output


class Topology(AirconicsCollection):
    """Class to define abstract aircraft topologies as extensible lists
    of lifting surfaces, enclosure, and propulsion type objects.

    Parameters
    ----------
    parts - dictionary
        Should contain the following,
            {name: (Part, arity)}

        i.e. the string 'name' values are presented as a tuple or list of:
            Part - TopoDS_Shape
                The shape

            arity - int
                the arity (number of descendant nodes) attached to part

        A warning is raised if arities are not provided, in which case
        arity is assumed to be zero

    Attributes
    ----------
    _Tree - list
        the list of LISP-like instructions (in the order they were called
        with AddPart)

    Notes
    -----
    - warning will be raised if no arities are provided

    example:
        # (Wing is an airconics Lifting Surface instace):
        aircraft = Topology(parts={'Wing': (Wing['Surface'], 2)})

    Although not enforced, parts should be added to this class recursively
    (from the top node first) to represent the aircraft's flattened
    topological tree suggested by Sobester [1]. It is the users
    responsibility to ensure the input nodes are a valid lisp tree for a
    correct graph to result (no checks are currently performed)


    See Also: AirconicsCollection

    References
    ----------
    [1] Sobester, A., “Four Suggestions for Better Parametric Geometries,”
        10th AIAA Multidisciplinary Design Optimization Conference,
        AIAA SciTech, American Institute of Aeronautics and Astronautics,
        jan 2014.
    """
    ComponentTypes = {"fuselage": [float] * 7,
                      "liftingsurface": [float] * 6 + [dict],
                      "engine": [float] * 3+}

    varNames = {'liftingsurface': ['X', 'Y', 'Z', 'ChordFactor', 'ScaleFactor', 'Rotation', 'Type'], 
                'fuselage': ['X', 'Y', 'Z', 'XScaleFactor', 'NoseLengthRatio', 'TailLengthRatio',
                             'FinenessRatio'],
                'mirror': [],
                # 'empty': []
                }

    def __init__(self, parts={},
                 construct_geometry=True,
                 SimplificationReqd=True
                 ):

        # Start with an empty parts list, as all parts will be added using
        # the for loop of self[name] = XXX below (__setitem__ calls the base)
        # AirconicsCollection __setitem__, which adds part to self._Parts)
        super(Topology, self).__init__(parts=parts,
                                       mirror=False,
                                       SimplificationReqd=SimplificationReqd,
                                       )

        # Carry around the tree for visualisation purposes:
        self._deap_tree = None   # This will be populated when

        self.nparts = 0
        self.routine = None

        # This will allow components to track what they should be attached to
        # (Needs to be ordered so that the final entries can be removed)
        self.parent_nodes = OrderedDict()

        for name, part_w_arity in parts.items():
            self[name] = part_w_arity

    def __setitem__(self, name, part_w_arity):
        """Overloads the assignment operator used by AirconicsCollection
        to allow only tuples as inputs - arity must be specified for
        topology.

        Parameters
        ----------
        name - string
        part_w_arity - tuple
            (Airconics class, int), eg: (Fuselage, 2) is a Fuselage shape with
            2 descendents in its topological tree

        Notes
        -----
        appends to the self.Tree and self._OrderedParts attributes
        """
        try:
            part, arity = part_w_arity
        except:
            log.warning("no arity set. Treating as zero")
            part = part_w_arity
            arity = 0

        if len(self.parent_nodes) > 0:
            if self.parent_nodes.values()[-1] > 0:
                self.parent_nodes[self.parent_nodes.keys()[-1]] -= 1
            while len(self.parent_nodes) >= 1 and self.parent_nodes.values()[-1] == 0:
                self.parent_nodes.popitem()

        if arity > 0:
            self.parent_nodes[name] = arity

        super(Topology, self).__setitem__(name, part)

    def __str__(self):
        """Overloads the base classes string representation to resemble
        the Aircrafts flattened tree topology resembling a LISP tree.

        Currently only set up to allow a single mirror terminal

        Notes
        -----

        See also:
        """
        return str(self._deap_tree)

    def _reset(self):
        self._Parts = {}
        self._deap_tree = None
        self.parent_nodes.clear()
        self.nparts = 0
        self.mirror = False

    # def fit_to_parent(self, oldscaling, oldx, oldy, oldz):
    #     """Given an old scaling and old position (this will be a random number
    #     between 0 and 1 using the DEAP tree I have set up elsewhere), a new
    #     scaling and position is returned allowing the resulting shape to 'fit'
    #     to its parent"""
    #     # Need to get a scaling from the parent of arbitrary type:
    #     # using a try-except to work for any parent type ... could probably
    #     # do better here
    #     parent = self[self.parent_nodes.keys()[-1]]

    #     parent.fit_scale_location(oldscaling, oldx, oldy, oldz)

    #     try:
    #         # The LiftingSurface branch (could probably do better here)
    #         parentscalefactor = parent.ScaleFactor

    #     except AttributeError:
    #         # The Fuselage branch
    #         parentscalefactor = parent.Scaling[0]

    #     try:
    #         # The interpolation along the curve: this could be done better
    #         # Essentially, this is the length of the vector normalised by
    #         # the diagonal of a 1 by 1 by 1 cube
    #         curve = parent.LECurve.GetObject()
    #         interp_C = np.linalg.norm([oldx, oldy, oldz]) / np.sqrt(3)
    #         newapex = curve.Value(interp_C)
    #         newx, newy, newz = newapex.X(), newapex.Y(), newapex.Z()

    #     except AttributeError:
    #         # The Fuselage branch
    #         parent_apex = parent.BowPoint
    #         # dL = gp_Vec(parent.BowPoint, parent.SternPoint)
    #         # self._testpoints.append(parent.SternPoint)
    #         xmin, ymin, zmin, xmax, ymax, zmax = parent.Extents()

    #         newx = parent_apex.X() + (xmax - xmin) * oldx
    #         newy = parent_apex.Y() + (ymax - ymin) / 2. * oldy
    #         newz = parent_apex.Z() + (zmax - zmin) / 2. * oldz

    #         # self._testpoints.append(gp_Pnt(parent_x + xlength, parent.BowPoint.Y(), parent.BowPoint.Z()))



    #     # Ensure that the oldscaled and oldposition fractions does give
    #     # something invisibly small:
    #     # REMOVING THIS TEMPORARILY: trying to see if scalings > 1 are allowed
    #     # oldscaling = np.interp(oldscaling, [0, 1], scalingrange)

    #     # The scaling is some percentage of parent (assumes components get
    #     # smaller)
    #     newscaling = oldscaling * parentscalefactor


    #     return newscaling, newx, newy, newz

    @wrap_shapeN
    def mirrorN(self, *args):
        self.mirror = True

        if len(self.parent_nodes) > 0:
            # The arity of a mirror node must be added to a parent node so that
            # further subcomponents will be added to their parent, e.g.,
            # fuselage1(mirror2(lsurf0, lsurf0)) requires that (2-1) is added
            # to the number of subcomponents attached to the fuselage
            self.parent_nodes[self.parent_nodes.keys()[-1]] += len(args) - 1

        for arg in args:
            arg()

        self.mirror = False

        return None

    @wrap_shapeN
    def fuselageN(self, NoseX, NoseY, NoseZ, ScalingX, NoseLengthRatio,
                  TailLengthRatio, FinenessRatio, *args):
        """Parameter descriptions can be found from the airconics.Fuselage
        class, all are floats"""
        # Need to add constraints here so that the Fuselage has a good chance
        # of successful lofting: any out of range values will be moved to the
        # nearest boundary:
        arglimits = {NoseLengthRatio: (0.18, 0.19),
                     TailLengthRatio: (0.29, 0.295),
                     ScalingX: (0.5, 5),
                     FinenessRatio: (0.5, 2)}

        # NoseLengthRatio = np.interp(NoseLengthRatio, [0,1], arglimits[NoseLengthRatio])
        # TailLengthRatio = np.interp(TailLengthRatio, [0,1],
        # arglimits[TailLengthRatio])

        # For now, I'm fixing these values to avoid errors (curve projection
        # seems to fail)
        NoseLengthRatio = 0.182
        TailLengthRatio = 0.293

        # Fit shape to parent, if one is available
        if len(self.parent_nodes) > 0:
            parent = self[self.parent_nodes.keys()[-1]]

            ScalingX, NoseX, NoseY, NoseZ = parent.fit_scale_location(
                ScalingX, NoseX, NoseY, NoseZ)

        else:
            ScalingX = np.interp(ScalingX, [0, 1], arglimits[ScalingX])
            NoseX = NoseY = NoseZ = 0

        ScalingYZ = ScalingX / FinenessRatio
        
        # Fits N new components to this box layout
        fus = Fuselage(NoseLengthRatio=NoseLengthRatio,
                       TailLengthRatio=TailLengthRatio,
                       Scaling=[ScalingX, ScalingYZ, ScalingYZ],
                       NoseCoordinates=[NoseX, NoseY, NoseZ],
                       SimplificationReqd=self.SimplificationReqd
                       )
        # Do no be confused between the numbering of components and the number
        # of descendent nodes
        name = 'fuselage{}_{}'.format(len(args), len(self))
        self[name] = fus, len(args)

        if self.mirror:
            super(Topology, self).__setitem__(
                name + '_mirror', fus.MirrorComponents(plane='xz'))

        for arg in args:
            arg()

        return None

    @wrap_shapeN
    def liftingsurfaceN(self, ApexX, ApexY, ApexZ, ChordFactor, ScaleFactor,
                        Rotation, functional_params_dict, *args):
        # ScaleFactor = np.interp(ScaleFactor, [0,1], [1,50])

        NSeg = 21

        # Checks if the current shape is being fitted to a parent; otherwise
        # this is the root component to which all others will be 'fitted',
        # in which case a wing is created with its apex at the origin
        if len(self.parent_nodes) > 0:
            parent = self[self.parent_nodes.keys()[-1]]
            ScaleFactor, ApexX, ApexY, ApexZ = parent.fit_scale_location(
                ScaleFactor, ApexX, ApexY, ApexZ)

        else:
            ApexX = ApexY = ApexZ = 0

        P = (ApexX, ApexY, ApexZ)

        # Instantiate the class
        wing = liftingsurface.LiftingSurface(P,
                                             SegmentNo=NSeg,
                                             ScaleFactor=ScaleFactor,
                                             ChordFactor=ChordFactor,
                                             **functional_params_dict)

        # Rotate the component if necessary:
        # if surfacetype in ['AirlinerFin', 'StraightWing']:
        # , 90]) # V tail or vertical fin
        Rotation_deg = np.interp(Rotation, [-1, 0, 1], [-90, 0, 90])
        RotAx = gp_Ax1(gp_Pnt(*P), gp_Dir(1, 0, 0))
        wing.RotateComponents(RotAx, Rotation_deg)
        wing.LECurve.GetObject().Rotate(RotAx, np.radians(Rotation_deg))

        self['liftingsurface{}_{}'.format(
            len(args), len(self))] = wing, len(args)

        if self.mirror:
            super(Topology, self).__setitem__(
                'liftingsurface{}_{}_mirror'.format(
                    len(args), len(self) - 1), wing.MirrorComponents(plane='xz'))

        for arg in args:
            arg()

        return None

    @wrap_shapeN
    def engineN(self, SpanStation, XChordFactor, YZLengthRatio, CtrBelowLE, CtrForwardLE
        Rotation, Invert, *args):
        """
        parameters
        ----------
        SpanStation: float in [0, 1]
            The spanwise percentage at which to obtain the chord

        PylonLength: 

        YZLengthRatio:

        CtrBelowLE: float
            fractional distance from hchord to the engine in engine nacelle lengths

        Rotation : Rotation angle around the chord fitting point

        Invert : int or bool (1 or 0)
            project above (0) or below (1) the xy plane

        Engines are always in the x direction, hence y-length cuts
        """
        # Allowing these to be fixed for now
        arglimits = {XChordFactor: (0.25, 1.0),
                     YZLengthRatio: (0.25, 1.0)}


        Scarf_deg = 0

        XChordFactor = np.interp(XChordFactor, [0, 1], arglimits[XChordFactor])
        YZLengthRatio = np.interp(XChordFactor, [0, 1], arglimits[YZLengthRatio])

        if len(self.parent_nodes) > 0:
            parent = self[self.parent_nodes.keys()[-1]]
            # obtain chord for fitting engine to - this is different for a 
            # wing and a fuselage
            SpanStation = SpanStation
            HChord = parent.get_spanstation_chord(SpanStation)
            Chord = HChord.GetObject()
            CEP = Chord.EndPoint()
            NacelleLength = XChordFactor * (CEP.Distance(Chord.StartPoint()))
        else:
            HChord = 0
            NacelleLength = 1

        # interpolate values between allowable limits

        EngineDia = YZLengthRatio * NacelleLength
        EngineCtrBelowLE = CtrBelowLE * NacelleLength
        EngineCtrFwdOfLE = CtrForwardLE * NacelleLength

        Centreloc = [CEP.X()-EngineCtrFwdOfLE*NacelleLength,
                 CEP.Y(),
                 CEP.Z()-EngineCtrBelowLE*NacelleLength]

        #   Now build the engine and its pylon
        eng = Engine(HChord,
                      CentreLocation=Centreloc,
                      ScarfAngle=Scarf_deg,
                      HighlightRadius=EngineDia/2.0,
                      MeanNacelleLength=NacelleLength)

        # Rotate the engine around the hchord (180deg range?):
        # RotAx = gp_Ax1(gp_Pnt(*P), gp_Dir(1, 0, 0))
        eng.RotateComponents(HChord, np.radians(Rotation_deg))
        Rotation_deg = np.interp(Rotation, [-1, 1], [-90 90])

        self['engine{}_{}'.format(
            len(args), len(self))] = eng, len(args)

        if self.mirror:
            super(Topology, self).__setitem__(
                'engine{}_{}_mirror'.format(
                    len(args), len(self) - 1), eng.MirrorComponents(plane='xz'))

        for arg in args:
            arg()        


        # if parent is fuselage, standard pylon plane should be horizontal,
        # otherwise if it's a wing, fit it vertically
        return None


    def pydot_graph(self):
        """Returns a pydot Graph for visualizing the topology tree.

        Currently only set up to allow a single mirror terminal

        Returns
        -------
        output : pydot.Dot
            The pydot object containing a tree representation of the program.

        Notes
        -----
        Can be visualised with pydot,

        :Example:
            >>> topo = Topology()     # Add some parts with topo.addPart
            >>> graph = topo.pydot_graph()
            >>> Image(graph.create_png())

        May add a dependency on GPLearn later and overload the appropriate
        class methods.
        """
        # Use the _deap_Tree if one is available (stored on calling run(tree,
        # pset)):
        if self._deap_tree:
            # Note: ns below is a simple range list for every edge/label
            nodes, edges, labels = gp.graph(self._deap_tree)
        else:
            raise AttributeError(
                "No DEAP GP tree was found: see Box_Layout.run(tree, pset)")

        mirror_flag = False

        graph = pydot.Dot(ranksep='0.1', nodesep='0.1')
        graph.set_node_defaults(style='filled')
        graph.set_edge_defaults(arrowhead='none')

        cluster_1 = pydot.Cluster('standard', color='invis')
        graph.add_subgraph(cluster_1)

        N_mirrored = 0

        # Add a sub cluster for mirrored objects (does nothing if empty)
        cluster_2 = pydot.Cluster('mirrored', style='dashed')

        # i is used to increment the number of Geometric parts that have been
        # found in the main loop (required as component names are prepended
        # with their ordinal component number, and component names need to be
        # matched with the _deap_tree)
        i = 0
        # use a boolean flag to decide if subclusters should be added to
        # the cluster of mirrored components (clutser_2)
        mirror_flag = False

        for node in nodes:
            # note: this only works because the node number is manually added
            # to the label by boxn:

            label = labels[node]

            try:
                # if label is a string, get the name of the function (remove
                # arity value at the end of the string)
                nodetype = label.rstrip('0123456789')

                if nodetype in NODE_PROPERTIES:
                    if (nodetype == 'mirror'):
                        # Need to expect some direct subcomponents to be added to
                        # the cluster_2
                        arity = label.lstrip(nodetype)
                        N_mirrored += int(arity)

                    elif (N_mirrored > 0):
                        # If direct subcomponents have further levels of recursion,
                        # need to add the next <arity> number of components to
                        # cluser_2
                        arity = label.lstrip(nodetype)
                        N_mirrored += int(arity) + \
                            len(self.ComponentTypes[nodetype])
                else:
                    # if the nodetype is a string, but not a known nodetype,
                    # then assume it's a lifting surface parametric function:
                    nodetype = 'function'

            except AttributeError:
                if isinstance(label, float):
                    # Assume the value is a float
                    label = "{}".format(label)
                nodetype = 'number'

            pydot_node = pydot.Node(
                node, label=label, **NODE_PROPERTIES[nodetype])

            if mirror_flag:
                cluster_2.add_node(pydot_node)
                N_mirrored -= 1
            else:
                cluster_1.add_node(pydot_node)

            if nodetype == "mirror":
                mirror_flag = True
            elif N_mirrored == 0:
                mirror_flag = False

        graph.add_subgraph(cluster_2)

        for edge in edges:
            src, dest = edge
            pydot_edge = pydot.Edge(src, dest)
            graph.add_edge(pydot_edge)

        return graph

    def export_graphviz(self):
        """Returns a string, Graphviz script for visualizing the topology tree.

        This function is included for legacy purposes. The preferred method is
        to use Topology.pydot_graph.

        Returns
        -------
        output : string
            The Graphviz script to plot the tree representation of the program.

        Notes
        -----
        Can be visualised with pydot,

        :Example:
            >>> topo = Topology()     # Add some parts with topo.addPart
            >>> graph = pydot.graph_from_dot_data(topo.export_graphviz())
            >>> Image(graph.create_png())

        See also
        --------
        Topology.pydot_graph

        May add a dependency on GPLearn later and overload the appropriate
        class methods.
        """
        graph = self.pydot_graph()
        return graph.create_dot()

    def to_json(self):
        json_obj = []
        for node in self._deap_tree:
            if isinstance(node, gp.Primitive):
                nodetype = re.sub(r'\d+', '', node.name)
                json_obj.append({'primitive': node.name})
                cycle = itertools.cycle(self.varNames[nodetype])
                json_obj[-1]['args'] = {}
            else:
                # try:
                json_obj[-1]['args'][next(cycle)] = node.value
                # except StopIteration:
                #     # This 
                #     pass

        return json_obj

    def write_json(self, fname='layout.json'):
        """
        """
        json_obj = self.to_json()
        with open(fname, 'w') as fout:
            json.dump(json_obj, fout, indent=2)


    def AddPart(self, part, name, arity=0):
        """Overloads the AddPart method of AirconicsCollection base class
        to append the arity of the input topology node

        Parameters
        ----------
        part - LiftingSurface, Engine or Fuselage class instance
            the part to be added to the tree

        name - string
            name of the part (will be used to look up this part in
            self.aircraft)

        arity - int
            The number of terminals attached to this part; this will be
            randomized at a later stage

        Notes
        -----
        This method is expected to be used recursively, therefore
        the order in which parts are added dictates the tree topology.
        The first item added will be the top of the tree.

        See also: AirconicsCollection.AddPart
        """
        self.__setitem__(name, (part, arity))


class Topology_GPTools(object):
    """
    """

    def __init__(self, MaxAttachments=2, fitness_funct=default_fitness,
                 min_levels=2,
                 max_levels=4,
                 min_mut=1,
                 max_mut=3,
                 pset_name="MAIN",
                 SimplificationReqd=True,
                 fitness_weights=(1.0,),
                 tournsize=5,
                 history=False):
        self.MaxAttachments = MaxAttachments
        self.min_levels = min_levels
        self.max_levels = max_levels
        self.SimplificationReqd = SimplificationReqd
        self.min_mut = min_mut
        self.max_mut = max_mut
        self.history = None

        self._pset = self.create_pset(name=pset_name)
        self._toolbox, self._creator = self.create_toolbox(
            fitness_weights=fitness_weights, min_levels=min_levels,
            max_levels=max_levels, tournsize=tournsize, history=history)

        self._topology = None

        # Need to bind the fitness function to the object here:
        self.fitness_funct = types.MethodType(wrap_fitnessfunct(fitness_funct), self)

    def run(self, tree):
        # This function currently overwrites the existing topology attribute
        # with a new topology. Speed increase may be found here by reusing the
        # Topology with self._topology._reset(), however, this can cause
        # unexpected behaviour with references returned by previous runs
        self._topology = Topology(SimplificationReqd=self.SimplificationReqd)
        routine = gp.compile(tree, self._pset)
        self._topology._deap_tree = tree
        routine()
        return self._topology

    def mirrorN(self, *args):
        """Passes args obtained from the GP primitive set attributed to this
        object to the CURRENT _topology attribute.

        This function is required as _topology is overwritten on every run, and
        hence its mirrorN methods are replaced on every run, invalidating the
        primitive set.

        See Also
        --------
        airconics.Topology.mirrorN
        """
        return self._topology.mirrorN(*args)

    def liftingsurfaceN(self, *args):
        """Passes args obtained from the GP primitive set attributed to this
        object to the CURRENT _topology attribute.

        This function is required as _topology is overwritten on every run, and
        hence its liftingSurfaceN methods are replaced on every run: the
        primitive set must use the current topologies liftingsurfaceN method.

        See Also
        --------
        airconics.Topology.liftingsurfaceN
        """
        return self._topology.liftingsurfaceN(*args)

    def fuselageN(self, *args):
        """Passes args obtained from the GP primitive set attributed to this
        object to the CURRENT _topology attribute.

        This function is required as _topology is overwritten on every run, and
        hence its fuselageN methods are replaced on every run: the
        primitive set must use the current topologies fuselageN method.

        See Also
        --------
        airconics.Topology.fuselageN
        """
        return self._topology.fuselageN(*args)

    def create_pset(self, name="MAIN"):
        """Creates the primitive set to be used for compiling topology 'programs'

        Parameters
        ----------
        names : string
            The name of the primitive set (unique identifer for this primitive
            set, should be different between objects)

        Returns
        -------
        pset : gp.PrimitiveSetTyped
            The set of primitive functions and terminals by which a gp program
            tree can be created

        See Also
        --------
        deap.gp.PrimitiveSetTyped
        """
        pset = gp.PrimitiveSetTyped(name, [], functools.partial)

        # Automatically add primitive for each type with integer numbers of
        # 'attached' subcomponents up to MaxAttachments (__init__ argument)
        for comptype, argtypes in Topology.ComponentTypes.items():
            # For now, engines are added as leaves (i.e. engine0) only
            full_argtypes = argtypes + functools
            pset.addPrimitive(getattr(self, 'engineN'), ar)

            for i in range(self.MaxAttachments + 1):
                name = comptype
                # get Number of inputs of the basic method e.g. fuselageN, and
                # add N (-1 due to self) float arguments to the typed
                # primitive
                full_argtypes = argtypes + [functools.partial] * i
                pset.addPrimitive(getattr(self, name + 'N'),
                                  full_argtypes, functools.partial,
                                  name=name + str(i))

        # Adding a leaf_primitives list to pset: these are the names of prims
        # that DO NOT CONTAIN SUBCOMPONENTS: the second to last level of
        # primitives in a tree must be one of these (forced in 'generate'
        # function)
        pset.leaf_primitives = {functools.partial: ['fuselage0', 'liftingsurface0', 'engine0'],
        float: [], dict: []}

        # mirroring primitives (need to start from mirror1 to avoid bloat)
        for i in range(1, self.MaxAttachments + 1):
            name = 'mirror' + str(i)
            pset.addPrimitive(
                self.mirrorN, [functools.partial] * i, functools.partial, name=name)

        # Primitives for defining shape of lifting surfaces:
        for wingtype, params in LSURF_FUNCTIONS.items():
            pset.addTerminal(params, dict, name=wingtype)

        # Workarounds: gp.generate doesn't seem to like mid-tree
        # terminals, so for now just add some primitive operators that
        # do a similar thing as the terminals:
        # def random_lsurfdict():
            # return np.random.choice(LSURF_FUNCTIONS.values())

        # pset.addEphemeralConstant('random_lsurf', random_lsurfdict, dict)

        # for wingtype, params in LSURF_FUNCTIONS.items():
        #     for funct_name, function in params.items()[:-1]:
        #         name=funct_name + "_" + wingtype
        #         pset.addPrimitive(function, [], types.FunctionType, name=name)
        #     # The last function is expected to be an airfoil function
        #     name, function=params.items()[-1]

        # def empty():
        #     """This is workaround function: see comment above"""
        #     return None

        # pset.addTerminal(empty, types.NoneType)

        # pset.addPrimitive(np.random.rand, [], float)

        pset.addEphemeralConstant('rand', random.random, float)

        return pset

    def create_toolbox(self, fitness_weights=(1.0,), min_levels=2, max_levels=4,
                       tournsize=2, history=False):
        """

        Parameters
        ----------
        min, max : int
            The minimum and maximum numbers of levels of recursion

        tournsize : int
            Number of individuals to enter tournament selection

        fitness_weights : tuple (default (1.0,))
            the weighting of the fitness values. (1.,) find global max, (-1.0,)
            is a minimisation. See DEAP base.Fitness. Multiple objectives are
            possible.

        See Also
        --------
        deap.gp.tools, deap.base.Toolbox, deap.base.Fitness
        """
        if not self._pset:
            self._pset = self.create_pset()

        creator.create("Fitness", base.Fitness, weights=fitness_weights)
        creator.create("Individual", gp.PrimitiveTree,
                       fitness=creator.Fitness)
    
        toolbox = base.Toolbox()

        # Attribute generator
        toolbox.register("expr_init", generate_topology, pset=self._pset,
                         min_=min_levels, max_=max_levels)

        # Structure initializers
        toolbox.register("individual", tools.initIterate,
                         creator.Individual, toolbox.expr_init)
        toolbox.register("population", tools.initRepeat,
                         list, toolbox.individual)

        # The evolutionary operators
        toolbox.register("evaluate", self.evalTopology)  # This may cause errors with multiprocessing, due to self
        toolbox.register("select", tools.selTournament, tournsize=tournsize)
        toolbox.register("mate", gp.cxOnePoint)


        toolbox.register("expr_mut", generate_topology, min_=self.min_mut, max_=self.max_mut)
        toolbox.register("mutate", gp.mutUniform, expr=toolbox.expr_mut, pset=self._pset)

        if history:
            self.history = tools.support.History()
            toolbox.decorate("mate", self.history.decorator)
            toolbox.decorate("mutate", self.history.decorator)

        return toolbox, creator

    def randomize(self):
        """Generates a new aircraft topology (configuration), using a
        randomized component hierarchy

        See Also
        --------
        Topology._toolbox.individual
        """
        tree = self._toolbox.individual()
        return self.run(tree)

    def _init_population(self, n):
        self.population = self._toolbox.population(n)

    def optimize(self, n=30, cxpd=0.5, mutpd=0.2, ngen=20):
        """
        Parameters
        ----------
        n : int
            The population size

        cxpd : scalar
            Probably of mating two individuals

        mutpd : scalar
            Probability of mutation an individual

        ngen : int
            Number of generations

        Returns
        -------


        See Also
        --------
        deap.algorithms.eaSimple
        """
        self._init_population(n)
        if self.history:
            self.history.update(self.population)

        hof = tools.HallOfFame(100)
        stats = tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", np.mean)
        stats.register("std", np.std)
        stats.register("min", np.min)
        stats.register("max", np.max)

        population, logbook, gen_best = eaSimple_logbest(self.population, self._toolbox, cxpd, mutpd,
                            ngen, stats, halloffame=hof)

        # get the best individual and rerun it:
        best = hof[0]

        self.run(best)

        return population, logbook, hof, gen_best

    def evalTopology(self, individual):
        topo = self.run(individual)
        return self.fitness_funct(topo),

    def from_string(self, config_string=None, preset=None):
        """Generates a new aircraft topology from a parsed input string.

        The arities of each function passed in the input string must not
        be larger than the maximum number of components (MaxAttachments)
        passed to the class on contruction.

        Parameters
        ----------
        config_string : string
            The recursive parse string of components and inputs

        Notes
        -----
        An error will be raised if the number or type of inputs is not correct.
        See the primitive functions documentation for inputs and types.

        See Also
        --------
        Topology.fuselageN, Topology.liftingsurfaceN, Topology.mirrorN
        """
        if preset:
            preset = preset.lower()
            assert(preset in self.preset_strs),\
                "{} is not a known preset layouts. Choose from {}".format(preset, self.preset_strs.keys())
            config_string = self.preset_strs[preset]
        tree = gp.PrimitiveTree.from_string(config_string, self._pset)
        return self.run(tree)

    def from_json(self, json_array):
        """
        """
        expr = []

        for component in json_array:
            prim_name = component['primitive']
            basename = re.sub(r'\d+', '', prim_name)


            if basename != 'mirror':
                prim_args = component['args']
            else:
                # This is a mirror component, no args reqd.
                pass

            arity = int(re.findall('\d+$', prim_name)[0])
            if prim_name in self._pset.mapping:
                expr.append(self._pset.mapping[prim_name])

            for arg in Topology.varNames[basename]:
                assert(arg in prim_args), \
                    'Input file does not contain parameter {} for component of type {}'. format(arg, prim_name)
                if prim_args[arg] in self._pset.mapping:
                    terminal = self._pset.mapping[prim_args[arg]]
                    expr.append(terminal)
                else:
                    # Probably a floating point number or string
                    type_ = type(prim_args[arg])

                    expr.append(gp.Terminal(prim_args[arg], False, type_))

        tree = self._creator.Individual(expr)
        return self.run(tree)

    def from_file(self, fname, loader='json'):
        """Opens file 'fname' and attempts to load the Topology described
        within.

        Currently only supports json files.

        Parameters
        ----------
        fname : string

        loader : string (default 'json')
            Defines the loading method used to extract the files contents.
            Currently only allows json

        Notes
        -----
        the 'loader' parameter is used instead of a file extension method as
        JSON formatted files do not require the extension to be '.json'

        See Also
        --------
        from_JSON
        """
        if loader == "json":
            with open(fname, 'r') as fin:
                json_array = json.load(fin)
            return self.from_json(json_array)
        else:
            raise ValueError(
                "{} is not a known file loading method".format(loader))
