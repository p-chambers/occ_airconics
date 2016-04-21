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
import copy
import numpy as np

# This dictionary will be used for topology tree formatting
FUNCTIONS = {'E': Fuselage,         # E = Enclosure
             'L': LiftingSurface,   # L = Lifting Surface
             'P': Engine}           # P = Propulsion

# Reversed dictionary for manually adding shapes, i.e. converting
#  a class instance to a string 
FUNCTIONS_INV = {func: name for name, func in FUNCTIONS.items()}

#The shapes of nodes in the exported graph from Topo class:
SHAPES = {'E': 'ellipse',
          'L': 'box',
          'P': 'hexagon'}

class TreeNode(object):
    def __init__(self, part, name, affinity):
        """Basic type to define node elements in the topology tree. To be used
        by Topology class.
        Parameters
        ----------
        part - Airconics type Fuselage, LiftingSurface or Engine
            The type to convert 
        
        name - string
            The name of the part (e.g. 'Wing' or 'Fin')
        
        affinity - int
            the number of descendant                
        
        Attributes
        ----------
        affty - int
            Affinity (number of descendants) of this node
            
        name - string
            Name of the part
            
        func - string
            Indicates the type of node i.e.
        """
        self.affty = affinity
        self.name = name
        if type(part) not in FUNCTIONS.values():
            raise TypeError("Not a recognised part type: {}. Should be {}"
                .format(type(part), FUNCTIONS.values()))
        else:
            func_str = FUNCTIONS_INV[type(part)]
        self.func = FUNCTIONS_INV[type(part)]
        
        def __str__(self):
            output = (self.name, self.func, self.affty)
            return output

class Topology(AirconicsCollection):
    def __init__(self, parts={}):
        """Class to define abstract aircraft topologies as extensible lists
        of lifting surfaces, enclosure, and propulsion type objects. 
        
        Parameters
        ----------
        parts - dictionary
            Should contain the following,
                {name: (Part, affinity)}
            i.e. the string 'name' values are presented as a tuple or list of:
                Part - TopoDS_Shape
                    The shape
                affinity - int
                    the affinity (number of descendant nodes) attached to part
            A warning is raised if afinities are not provided, in which case
            affinity is assumed to be zero
        
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
        self._Tree = []

        for name, part_w_affinity in parts.items():
            
            try:
                part, affinity = part_w_affinity
            except:
                print("Warning: no affinity set. Treating as zero")
                part = part_w_affinity
                affinity = 0

            node = TreeNode(part, name, affinity)
            self._Tree.append(node)            

        # This stores the (name: part) into the self using base class init
        super(Topology, self).__init__(parts=parts)
        
    def __setitem__(self, name, part_w_affinity):
        """Overloads the assignment operator used by AirconicsCollection
        to allow only tuples as inputs - Affinity must be specified for
        topology.
        
        Parameters
        ----------
        name - string
        part_w_affinity - tuple
            (Airconics class, int), eg:
                (Fuselage, 2) is a Fuselage shape with 2 descendents in 
                its topological tree
        
        Notes
        -----
        appends to the self.Tree and self._OrderedParts attributes 
        """
        try:
            part, affinity = part_w_affinity
        except:
            print("Warning: no affinity set. Treating as zero")
            part = part_w_affinity
            affinity = 0            

        node = TreeNode(part, name, affinity)
        self._Tree.append(node)
        super(Topology, self).__setitem__(name, part)

    def __str__(self):
        """Overloads the base classes string representation to resemble
        the Aircrafts flattened tree topology resembling a LISP tree.
        Notes
        -----
        This follows a similar format to GPLearn program representation
        
        See also: self.__init__ notes
        """
        terminals = [0]
        output = ''
        
        for i, node in enumerate(self._Tree):
#            If node has a non zero affinity, there is some nested printing
#            required, otherwise node is a terminal:
            if node.affty > 0:
                terminals.append(node.affty)
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

#    def _Levels(self):
#        """Calculates the number of levels of recursion of the current
#        topological tree
#        """
#        n_level = 1             # A program must have at least 1 level
#        for node in self._Tree:
#            func, affty = node
#            if affty > 0:
#                n_level += 1
#        return depth
        
    def export_graphviz(self):
        """Returns a string, Graphviz script for visualizing the program.
        Returns
        -------
        oupput : string
            The Graphviz script to plot the tree representation of the program.
            
        Notes
        -----
        This function is originally from GPLearns _Program class, but has been
        modified. 
        
        May add a dependency on GPLearn later and overload the appropriate
        class methods.
        """
        terminals = []
        output = "digraph program {\nnode [style=filled]"
        for i, node in enumerate(self._Tree):
            fill = "#136ed4"
            if node.affty > 0:
                terminals.append([node.affty, i])
                output += ('%d [label="%s", fillcolor="%s", shape="%s"] ;\n'
                           % (i, node.name, fill, SHAPES[node.func]))
            else:
                output += ('%d [label="%s", fillcolor="%s", shape="%s"] ;\n'
                           % (i, node.name, fill, SHAPES[node.func]))
                if i == 0:
                    # A degenerative program of only one node
                    return output + "}"
                terminals[-1][0] -= 1
                terminals[-1].append(i)
                while terminals[-1][0] == 0:
                    output += '%d -> %d ;\n' % (terminals[-1][1],
                                                terminals[-1][-1])
                    terminals[-1].pop()
                    if len(terminals[-1]) == 2:
                        parent = terminals[-1][-1]
                        terminals.pop()
                        if len(terminals) == 0:
                            return output + "}"
                        terminals[-1].append(parent)
                        terminals[-1][0] -= 1

        # We should never get here
        return None
        
    def AddPart(self, part, name, affinity=0):
        """Overloads the AddPart method of AirconicsCollection base class
        to append the affinity of the input topology node
        
        Parameters
        ----------
        name - string
            name of the part (will be used to look up this part in
            self.aircraft)
            
        part - LiftingSurface, Engine or Fuselage class instance
            the part to be added to the tree
        
        affinity - int
            The number of terminals attached to this part; this will be
            randomized at a later stage
        
        Notes
        -----
        This method is expected to be used recursively, therefore
        the order in which parts are added dictates the tree topology. 
        The first item added will be the top of the tree.
        
        See also: AirconicsCollection.AddPart
        """
        self.__setitem__(name, (part, affinity))
