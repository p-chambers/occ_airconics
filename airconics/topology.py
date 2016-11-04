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
from OCC.gp import gp_Ax2, gp_Ax1, gp_Dir, gp_Pnt

import types
import functools
from functools import partial
import pydot
from collections import OrderedDict
import numbers
import numpy as np
from deap import algorithms
from deap import base
from deap import creator
from deap import tools
from deap import gp

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
                   'rand': {'shape': 'ellipse'},
                   'None': {'shape': 'point'},
                   }

LSURF_FUNCTIONS = {'AirlinerWing':
   OrderedDict([('SweepFunct', (mySweepAngleFunctionAirliner,
                                types.FunctionType)),
                ('DihedralFunct', (myDihedralFunctionAirliner,
                 types.FunctionType)),
                ('ChordFunct',
                 (myChordFunctionAirliner, types.FunctionType)),
                ('TwistFunct',
                 (myTwistFunctionAirliner, types.FunctionType)),
                ('AirfoilFunct', (myAirfoilFunctionAirliner, Airfoil))
                ]),
   'AirlinerTP':
   OrderedDict([('SweepFunct', (mySweepAngleFunctionTP, types.FunctionType)),
                ('DihedralFunct',
                 (myDihedralFunctionTP, types.FunctionType)),
                ('ChordFunct', (myChordFunctionTP, types.FunctionType)),
                ('TwistFunct', (myTwistFunctionTP, types.FunctionType)),
                ('AirfoilFunct', (myAirfoilFunctionTP, Airfoil))
                ]),
   'AirlinerFin':
   OrderedDict([('SweepFunct', (mySweepAngleFunctionFin, types.FunctionType)),
                ('DihedralFunct',
                 (myDihedralFunctionFin, types.FunctionType)),
                ('ChordFunct', (myChordFunctionFin, types.FunctionType)),
                ('TwistFunct', (myTwistFunctionFin, types.FunctionType)),
                ('AirfoilFunct', (myAirfoilFunctionFin, Airfoil))
                ]),
   'StraightWing':
   OrderedDict([('SweepFunct', (SimpleSweepFunction, types.FunctionType)),
                ('DihedralFunct', (SimpleDihedralFunction, types.FunctionType)),
                ('ChordFunct', (SimpleChordFunction, types.FunctionType)),
                ('TwistFunct', (SimpleTwistFunction, types.FunctionType)),
                ('AirfoilFunct', (SimpleAirfoilFunction, Airfoil))
                ])
            }


# Use a wrapper to convert boxN, sphereN etc. to another function that
# returns a callable
def wrap_shapeN(shapeN):
    @functools.wraps(shapeN)
    def wrapped_shapeN(*args, **kwargs):
        return partial(shapeN, *args)
    return wrapped_shapeN


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
    try:
        xmin, ymin, zmin, xmax, ymax, zmax=topology.Extents()
    except:
        # Bounding Box was probably void
        return 0
    return (xmax - xmin) * (ymax - ymin) * (zmax - zmin)


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
        self.name=name
        self.arity=arity

        if type(part) not in FUNCTIONS.values():
            raise TypeError("Not a recognised part type: {}. Should be {}"
                            .format(type(part), FUNCTIONS.values()))
        else:
            func_str=FUNCTIONS_INV[type(part)]
        self.func=func_str

    def __str__(self):
        output='({}, {}, {})'.format(self.name, self.func, self.arity)
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
    ComponentTypes={"fuselage": [float] * 7,
                      "liftingsurface": [float] * 5 + [types.FunctionType] * 4 +
                                        [Airfoil]}

    def __init__(self, parts={},
                 MaxAttachments=2,
                 construct_geometry=True,
                 SimplificationReqd=True,
                 fitness_funct=default_fitness,
                 min_levels=2,
                 max_levels=4,
                 pset_name="MAIN"
                 ):

        # Start with an empty parts list, as all parts will be added using
        # the for loop of self[name] = XXX below (__setitem__ calls the base)
        # AirconicsCollection __setitem__, which adds part to self._Parts)
        super(Topology, self).__init__(parts=parts,
                                       MaxAttachments=MaxAttachments,
                                       mirror=False,
                                       _pset=None,
                                       _toolbox=None,
                                       SimplificationReqd=SimplificationReqd,
                                       )

        # Carry around the tree for visualisation purposes:
        self._deap_tree=None   # This will be populated when

        # Start with a simple box
        self.nparts=0
        self.routine=None

        # This will allow components to track what they should be attached to
        # (Needs to be ordered so that the final entries can be removed)
        self.parent_nodes=OrderedDict()

        # Need to bind the fitness function to the object here:
        self.fitness_funct=types.MethodType(fitness_funct, self)

        self._pset=self.create_pset(name=pset_name)
        self._toolbox=self.create_toolbox(min_levels, max_levels)

        for name, part_w_arity in parts.items():
            self[name]=part_w_arity

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
            part, arity=part_w_arity
        except:
            log.warning("no arity set. Treating as zero")
            part=part_w_arity
            arity=0

        if len(self.parent_nodes) > 0:
            if self.parent_nodes.values()[-1] > 1:
                self.parent_nodes[self.parent_nodes.keys()[-1]] -= 1
            else:
                while self.parent_nodes.values()[-1] < 1:
                    self.parent_nodes.popitem()

        self.parent_nodes[name]=arity

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
        self._Parts={}
        self._deap_tree=None
        self.parent_nodes.clear()
        self.nparts=0
        self.mirror=False

    def run(self, tree):
        self._reset()
        routine=gp.compile(tree, self._pset)
        self._deap_tree=tree
        routine()

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
        pset=gp.PrimitiveSetTyped(name, [], types.NoneType)

        # Automatically add primitive for each type with integer numbers of
        # 'attached' subcomponents up to MaxAttachments (__init__ argument)
        for comptype, argtypes in self.ComponentTypes.items():
            for i in range(self.MaxAttachments + 1):
                name=comptype
                # get Number of inputs of the basic method e.g. fuselageN, and
                # add N (-1 due to self) float arguments to the typed
                # primitive
                argtypes=argtypes + [types.NoneType] * i
                pset.addPrimitive(getattr(self, name + 'N'),
                                  argtypes, types.NoneType, name=name + str(i))

        # mirroring primitives (need to start from mirror1 to avoid bloat)
        for i in range(1, self.MaxAttachments + 1):
            name='mirror' + str(i)

            pset.addPrimitive(
                self.mirrorN, [types.NoneType] * i, types.NoneType, name=name)

        # Primitives for defining shape of lifting surfaces:
        for wingtype, params in LSURF_FUNCTIONS.items():
            for funct_name, (function, ret_type) in params.items():
                name = funct_name + "_" + wingtype
                pset.addTerminal(function, ret_type, name=name)
                print(name, funct_name, ret_type)

        # Workarounds: gp.generate doesn't seem to like mid-tree
        # terminals, so for now just add some primitive operators that
        # do a similar thing as the terminals:
        def random_lsurffunc():
            wingtype = np.random.choice(LSURF_FUNCTIONS.keys())
            lsurffunc = np.random.choice(LSURF_FUNCTIONS[wingtype].keys()[:-1])
            return LSURF_FUNCTIONS[wingtype][lsurffunc][0]

        def random_airfoilfunc():
            wingtype = np.random.choice(LSURF_FUNCTIONS.keys())
            airfoilfunc = LSURF_FUNCTIONS[wingtype].values()[-1]
            return airfoilfunc[0]

        pset.addPrimitive(random_lsurffunc, [], types.FunctionType)
        pset.addPrimitive(random_airfoilfunc, [], Airfoil)

        # for wingtype, params in LSURF_FUNCTIONS.items():
        #     for funct_name, function in params.items()[:-1]:
        #         name=funct_name + "_" + wingtype
        #         pset.addPrimitive(function, [], types.FunctionType, name=name)
        #     # The last function is expected to be an airfoil function
        #     name, function=params.items()[-1]

        def useless_None():
            """This is workaround function: see comment above"""
            return None

        pset.addTerminal(useless_None, types.NoneType)

        pset.addPrimitive(np.random.rand, [], float)

        pset.addEphemeralConstant('rand', np.random.rand, float)

        return pset

    def create_toolbox(self, fitness_method='max', min_=2, max_=4,
                       tournsize=7):
        """

        Parameters
        ----------
        min, max : int
            The minimum and maximum numbers of levels of recursion

        tournsize : int
            Number of individuals to enter tournament selection

        See Also
        --------
        deap.gp.tools, deap.base.Toolbox
        """
        if not self._pset:
            self._pset=self.create_pset()

        creator.create("FitnessMax", base.Fitness, weights=(1.0,))
        creator.create("Individual", gp.PrimitiveTree,
                       fitness=creator.FitnessMax)

        toolbox=base.Toolbox()

        # Attribute generator
        toolbox.register("expr_init", gp.genFull,
                         pset=self._pset, min_=min_, max_=max_)

        # Structure initializers
        toolbox.register("individual", tools.initIterate,
                         creator.Individual, toolbox.expr_init)
        toolbox.register("population", tools.initRepeat,
                         list, toolbox.individual)

        # The evolutionary operators
        toolbox.register("evaluate", self.evalTopology)
        toolbox.register("select", tools.selTournament, tournsize=tournsize)
        toolbox.register("mate", gp.cxOnePoint)
        toolbox.register("expr_mut", gp.genFull, min_=0, max_=2)
        toolbox.register("mutate", gp.mutUniform,
                         expr=toolbox.expr_mut, pset=self._pset)

        return toolbox

    def randomize(self):
        """Generates a new aircraft topology (configuration), using a
        randomized component hierarchy

        See Also
        --------
        Topology._toolbox.individual
        """
        tree=self._toolbox.individual()
        self.run(tree)

    def from_string(self, config_string):
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
        tree=gp.PrimitiveTree.from_string(config_string, self._pset)
        self.run(tree)

    def evalTopology(self, individual):
        self.run(individual, self._pset)
        return self.fitness_funct(),

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
        np.random.seed(69)
        pop=toolbox.population(n)
        hof=tools.HallOfFame(1)
        stats=tools.Statistics(lambda ind: ind.fitness.values)
        stats.register("avg", np.mean)
        stats.register("std", np.std)
        stats.register("min", np.min)
        stats.register("max", np.max)

        algorithms.eaSimple(pop, self._toolbox, 0.5, 0.2,
                            20, stats, halloffame=hof)

        # get the best individual and rerun it:
        best=hof[0]

        self.run(best)

        return pop, hof, stats

    def fit_to_parent(self, oldscaling, oldposition):
        """Given an old scaling and old position (this will be a random number
        between 0 and 1 using the DEAP tree I have set up elsewhere), a new
        scaling and position is returned allowing the resulting shape to 'fit'
        to its parent"""
        # Need to get a scaling from the parent of arbitrary type:
        # using a try-except to work for any parent type ... could probably
        # do better here
        parent=self[self.parent_nodes.keys()[-1]]
        scalingrange=(0.6, 1)

        # Ensure that the oldscaled and oldposition fractions does give
        # something invisibly small:
        oldscaling=np.interp(oldscaling, [0, 1], scalingrange)
        try:
            # The LiftingSurface branch (could probably do better here)
            parentscalefactor=parent.ScaleFactor

        except AttributeError:
            # The Fuselage branch
            parentscalefactor=parent.Scaling[0]

        try:
            # The LiftingSurface branch (could probably do better here)
            parent_x=parent.ApexPoint.X()
            xlength=parent.ScaleFactor * parent.ChordFactor

        except AttributeError:
            # The Fuselage branch
            parent_x=parent.NoseCoordinates[0]
            xlength=(parent.SternPoint.X() - parent.BowPoint.X()
                       ) * (parentscalefactor / 55.902)
        # The scaling is some percentage of parent (assumes components get
        # smaller)
        newscaling=oldscaling * parentscalefactor

        newx=parent_x + xlength * oldposition

        return newscaling, newx

    def Build(self):
        """Recursively builds all sub components in the current topology tree
        if self.construct_geometry is true. Will also mirror components
        if a mirror node has been added, regardless of if construct_geometry
        is true.

        Uses the the Build method of all sub components. Any user defined
        classes must therefore define the Build method in order for this to
        work correctly.
        """
        if self.construct_geometry:
            log.debug("Building all geometries from Topology object")
            for name, part in self.items():
                part.Build()

        self.MirrorSubtree()

    @wrap_shapeN
    def mirrorN(self, *args):
        self.mirror=True
        for arg in args:
            arg()

        self.mirror=False
        return None

    @wrap_shapeN
    def fuselageN(self, NoseX, NoseY, NoseZ, ScalingX, NoseLengthRatio,
                  TailLengthRatio, FinenessRatio, *args):
        """Parameter descriptions can be found from the airconics.Fuselage
        class, all are floats"""
        # Need to add constraints here so that the Fuselage has a good chance
        # of successful lofting: any out of range values will be moved to the
        # nearest boundary:
        arglimits={NoseLengthRatio: (0.18, 0.19),
                     TailLengthRatio: (0.29, 0.295),
                     ScalingX: (0.5, 5),
                     FinenessRatio: (0.5, 2)}

        # NoseLengthRatio = np.interp(NoseLengthRatio, [0,1], arglimits[NoseLengthRatio])
        # TailLengthRatio = np.interp(TailLengthRatio, [0,1],
        # arglimits[TailLengthRatio])

        # For now, I'm fixing these values to avoid errors (curve projection
        # seems to fail)
        NoseLengthRatio=0.182
        TailLengthRatio=0.293

        # Essentially this checks if the current shape is being fitted to a
        # parent
        if len(self.parent_nodes) > 0:
            ScalingX, NoseX=self.fit_to_parent(ScalingX, NoseX)
        else:
            ScalingX=np.interp(ScalingX, [0, 1], arglimits[ScalingX])

        FinenessRatio=np.interp(
            FinenessRatio, [0, 1], arglimits[FinenessRatio])

        ScalingYZ=ScalingX / FinenessRatio

        NoseY=NoseZ=0

#         print(NoseLengthRatio, TailLengthRatio, Scaling)
        # Fits N new components to this box layout
        fus=Fuselage(NoseLengthRatio=NoseLengthRatio,
                       TailLengthRatio=TailLengthRatio,
                       Scaling=[ScalingX, ScalingYZ, ScalingYZ],
                       NoseCoordinates=[NoseX, NoseY, NoseZ],
                       SimplificationReqd=self.SimplificationReqd
                       )
        # Do no be confused between the numbering of boxes and the number of
        # descendent nodes: Each box needs a unique ID, so naming a box0
        # function "box0" replaces other shapes in this layout that are also
        # named box0
        name='fuselage{}_{}'.format(len(args), len(self))
        self[name]=fus, len(args)

        if self.mirror:
            super(Topology, self).__setitem__(
                name + '_mirror', fus.MirrorComponents(plane='xz'))

        for arg in args:
            arg()

        return None

    @wrap_shapeN
    def liftingsurfaceN(self, ApexX, ApexY, ApexZ, ScaleFactor, Rotation,
                        SweepFunct, DihedralFunct, ChordFunct, TwistFunct,
                        AirfoilFunct, *args):

        # ScaleFactor = np.interp(ScaleFactor, [0,1], [1,50])
        ChordFactor=1

        # Class definition
        NSeg=10

        # Essentially this checks if the current shape is being fitted to a
        # parent
        if len(self.parent_nodes) > 0:
            ScaleFactor, ApexX=self.fit_to_parent(ScaleFactor, ApexX)
        else:
            ApexX=0

        ApexZ=ApexY=0
        P=(ApexX, ApexY, ApexZ)

        # Instantiate the class
        wing=liftingsurface.LiftingSurface(P,
                                             SegmentNo=NSeg,
                                             ScaleFactor=ScaleFactor,
                                             ChordFactor=ChordFactor,
                                             SweepFunct=SweepFunct,
                                             DihedralFunct=DihedralFunct,
                                             TwistFunct=TwistFunct,
                                             ChordFunct=ChordFunct,
                                             AirfoilFunct=AirfoilFunct)

        # Rotate the component if necessary:
        # if surfacetype in ['AirlinerFin', 'StraightWing']:
        # , 90]) # V tail or vertical fin
        # rotation = np.random.choice([0, 32.5])
        RotAx=gp_Ax1(gp_Pnt(*P), gp_Dir(1, 0, 0))
        wing.RotateComponents(RotAx, Rotation)

        self['liftingsurface{}_{}'.format(
            len(args), len(self))]=wing, len(args)

        if self.mirror:
            super(Topology, self).__setitem__(
                'liftingsurface{}_{}_mirror'.format(
                    len(args), len(self) - 1), wing.MirrorComponents(plane='xz'))

        for arg in args:
            arg()

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
            nodes, edges, labels=gp.graph(self._deap_tree)
        else:
            raise AttributeError(
                "No DEAP GP tree was found: see Box_Layout.run(tree, pset)")

        mirror_flag=False

        graph=pydot.Dot(ranksep='0.1', nodesep='0.1')
        graph.set_node_defaults(style='filled')
        graph.set_edge_defaults(arrowhead='none')

        cluster_1=pydot.Cluster('standard', color='invis')
        graph.add_subgraph(cluster_1)

        N_mirrored=0

        # Add a sub cluster for mirrored objects (does nothing if empty)
        cluster_2=pydot.Cluster('mirrored', style='dashed')

        # i is used to increment the number of Geometric parts that have been
        # found in the main loop (required as component names are prepended
        # with their ordinal component number, and component names need to be
        # matched with the _deap_tree)
        i=0

        # use a boolean flag to decide if subclusters should be added to
        # the cluster of mirrored components (clutser_2)
        mirror_flag=False

        for node in nodes:
            # note: this only works because the node number is manually added
            # to the label by boxn:

            label=labels[node]

            try:
                # if label is a string, get the name of the function (remove
                # arity value at the end of the string)
                nodetype=label.rstrip('0123456789')
                # print(nodetype)
                print(nodetype)

                if nodetype in NODE_PROPERTIES:
                    if (nodetype == 'mirror'):
                        # Need to expect some direct subcomponents to be added to
                        # the cluster_2
                        arity=label.lstrip(nodetype)
                        N_mirrored += int(arity)

                    elif (N_mirrored > 0):
                        # If direct subcomponents have further levels of recursion,
                        # need to add the next <arity> number of components to
                        # cluser_2
                        arity=label.lstrip(nodetype)
                        N_mirrored += int(arity) + \
                            len(self.ComponentTypes[nodetype])
                else:
                    # if the nodetype is a string, but not a known nodetype,
                    # then assume it's a lifting surface parametric function:
                    nodetype='function'

            except AttributeError:
                if isinstance(label, float):
                    # Assume the value is a float
                    label="{}".format(label)
                nodetype='number'

            pydot_node=pydot.Node(
                node, label=label, **NODE_PROPERTIES[nodetype])

            if mirror_flag:
                cluster_2.add_node(pydot_node)
                N_mirrored -= 1
            else:
                cluster_1.add_node(pydot_node)

            if nodetype == "mirror":
                mirror_flag=True
            elif N_mirrored == 0:
                mirror_flag=False

        graph.add_subgraph(cluster_2)

        for edge in edges:
            src, dest=edge
            pydot_edge=pydot.Edge(src, dest)
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
        graph=self.pydot_graph()
        return graph.create_dot()

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
