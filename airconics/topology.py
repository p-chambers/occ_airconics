# -*- coding: utf-8 -*-
"""
Classses and functions used to define arbitrary aircraft topologies in
Airconics, inspired by the GPLearn API

Created on Fri Apr 15 12:24:32 2016

@author: pchambers
"""
from .base import AirconicsCollection
from airconics.liftingsurface import LiftingSurface
from airconics.fuselage_oml import Fuselage
from airconics.engine import Engine
from OCC.gp import gp_Ax2
# import copy
# import numpy as np


# This dictionary will be used for topology tree formatting
FUNCTIONS = {'E': Fuselage,         # E = Enclosure
             'L': LiftingSurface,   # L = Lifting Surface
             'P': Engine,           # P = Propulsion
             '|': gp_Ax2}           # M = Mirror Plane

# Reversed dictionary for manually adding shapes, i.e. converting
#  a class instance to a string
FUNCTIONS_INV = {func: name for name, func in FUNCTIONS.items()}

# The shapes of nodes in the exported graph from Topo class:
SHAPES = {'E': 'ellipse',
          'L': 'box',
          'P': 'hexagon',
          '|': ''}                  # Note: M does not have a node shape


# def generate_population(size, max_recursion):
#     """
#     Parameters
#     ----------
#     size : int
#         The size of the population

#     max_recursion : int
#         The maximum number of levels of recursion for each member in the
#         population
#     """
#     population = []
#     for i in range(size):
#         topo = Topology()

#         population.append(topo)


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
    - warning will be raised if no affinities are provided

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
    def __init__(self, parts={},
                 construct_geometry=False):

        self._Tree = []
        # Start with an empty parts list, as all parts will be added using
        # the for loop of self[name] = XXX below (__setitem__ calls the base)
        # AirconicsCollection __setitem__, which adds part to self._Parts)
        super(Topology, self).__init__(parts={},
                                       construct_geometry=construct_geometry)

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
            print("Warning: no arity set. Treating as zero")
            part = part_w_arity
            arity = 0

        node = TreeNode(part, name, arity)

        self._Tree.append(node)
        super(Topology, self).__setitem__(name, part)

    def __str__(self):
        """Overloads the base classes string representation to resemble
        the Aircrafts flattened tree topology resembling a LISP tree.

        Currently only set up to allow a single mirror terminal

        Notes
        -----
        This follows a similar format to GPLearn program representation

        See also: self.__init__ notes
        """
        terminals = [0]
        output = ''

        for i, node in enumerate(self._Tree):
#            If node has a non zero arity, there is some nested printing
#            required, otherwise node is a terminal:
            if node.func == '|':
                # Mirror lines are a special case: simply put a line an skip
                # to next iteration
                output += '|'
                continue
            if node.arity > 0:
                terminals.append(node.arity)
                output += node.func + '('
            else:
                output += node.func
                terminals[-1] -= 1
                while terminals[-1] == 0:
                    terminals.pop()
                    terminals[-1] -= 1   # This stops the while loop on last it
                    output += ')'
                if i != len(self) - 1:
                    output += ', '
        return output

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
            print("Building all geometries from Topology object")
            for name, part in self.items():
                part.Build()

        self.MirrorSubtree()

    def MirrorSubtree(self):
        """Mirrors the geometry where required, based on the current topology
        tree.

        Does nothing is no mirror plane has been added
        """
        mirror_plane = False
        for node in self._Tree:
            if mirror_plane:
                print(type(mirror_plane))
                mirrored = self[node.name].MirrorComponents(axe2=mirror_plane)
                # Store the mirrored part, without adding it to self._Tree:
                name_str = node.name + '_mirror'
                super(Topology, self).__setitem__(name_str, mirrored)
            if node.func == '|':
                'Mirroring around plane {}'.format(node.name)
                mirror_plane = self[node.name]

    def export_graphviz(self):
        """Returns a string, Graphviz script for visualizing the topology tree.

        Currently only set up to allow a single mirror terminal

        Returns
        -------
        output : string
            The Graphviz script to plot the tree representation of the program.

        Notes
        -----
        This function is originally from GPLearns _Program class, but has been
        modified. Can be visualised with pydot,

        :Example:
            >>> topo = Topology()     # Add some parts with topo.addPart
            >>> graph = pydot.graph_from_dot_data(topo.export_graphviz())
            >>> Image(graph.create_png())

        May add a dependency on GPLearn later and overload the appropriate
        class methods.
        """
        terminals = []
        # Assume the geometry is not mirrored at first
        mirror_flag = False
        output = """digraph program {
        splines=ortho;
        ranksep="0.1";
        node [style=filled]
        edge [arrowhead=none];

        # Start the primary cluster (assume only one mirror exists, leading
        # to two clusters)
        subgraph cluster_0 {
            color=invis;
        """
        for i, node in enumerate(self._Tree):
            if node.func == '|':
                # Mirror node branch
                output += '}\n'     # Close the primary cluster
                output += 'subgraph cluster_1{\nstyle=dashed\n'
                mirror_flag = True

            else:
                # All other functions, e.g., Enclosure, LiftingSurface,
                # Propulsion
                fill = "#136ed4"
                if node.arity > 0:
                    terminals.append([node.arity, i])
                    output += ('%d [label="%s", fillcolor="%s", shape="%s"] ;\n'
                               % (i, node.name, fill, SHAPES[node.func]))
                    # Add a point below to allow orthogonal branching in graphs
                    output += ('p%d [shape=point, color=none];\n' %(i))
                    output += ('%d -> p%d;\n' %(i, i))
                else:
                    output += ('%d [label="%s", fillcolor="%s", shape="%s"] ;\n'
                               % (i, node.name, fill, SHAPES[node.func]))
                    if i == 0:
                        # A degenerative program of only one node
                        return output + "}"
                    terminals[-1][0] -= 1
                    terminals[-1].append(i)
                    while terminals[-1][0] == 0:
                        output += 'p%d -> %d ;\n' % (terminals[-1][1],
                                                     terminals[-1][-1])
                        terminals[-1].pop()
                        if len(terminals[-1]) == 2:
                            parent = terminals[-1][-1]
                            terminals.pop()
                            if len(terminals) == 0:
                                # close current cluster and end digraph
                                return output + "}\n}"

                            terminals[-1].append(parent)
                            terminals[-1][0] -= 1

        # We should never get here
        return None

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
