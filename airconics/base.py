# -*- coding: utf-8 -*-
"""Base classes used by OCC_Airconics

Contains container classes (AirconicsBase) which behaves like a dictionary


Created on Mon Apr 18 10:26:22 2016

@author: pchambers
"""
from abc import abstractmethod
from collections import MutableMapping
import os
import AirCONICStools as act
from OCC.Graphic3d import Graphic3d_NOM_ALUMINIUM
from OCC.TopoDS import TopoDS_Shape
from OCC.StlAPI import StlAPI_Writer
from OCC.AIS import AIS_Shape


class AirconicsBase(MutableMapping, object):
    """Base container class from which other base classes are derived from.
    This is a an abstract base class and should not be used directly by users.

    Notes
    -----
    When properly defined in inherited functions, this class should behave like
    a dictionary.
    """
    @abstractmethod
    def __init__(self, *args, **kwargs):
        pass

    @abstractmethod
    def __str__(self, *args, **kwargs):
        pass

    @abstractmethod
    def Display(self, *args, **kwargs):
        pass

    @abstractmethod
    def Write(self, *args, **kwargs):
        pass


class AirconicsShape(AirconicsBase):
    """Base class from which airconics parts will be made. This is a base
    class only and should not be used directly by users.

    Parameters
    ----------
    components - dictionary of components to be added as attributes
        To add attributes directly. Values must be OCC.TopoDS.TopoDS_Shape

    **kwargs - All other keyword arguments will be added as an attribute
        to the resulting class calling super(subclass, self).__init__

    Attributes
    ----------
    __Components - Dictionary of name(string):component(TopoDS_Shape) pairs
        This should not be interacted with directly. Use assignment or
        self.AddComponents (see Notes for more detail)

    Notes
    -----
    example:
        shape = DerivedShapeClass()
        shape['wing'] = wing  # OR
        shape.AddComponent(wing, 'WingSurface')

    * Derived classes should call the AirconicsCollection init with eg:
        super(DerivedClass, self).__init__(self, *args, **kwargs)"""

    def __init__(self, components={}, *args, **kwargs):
        # Set the components dictionary (default empty)
        self.__Components = {}
        for name, component in components.items():
            self.__setitem__(name, component)

        # Set all kwargs as attributes
        for key, value in kwargs.items():
            self.__setattr__(key, value)

    def __getitem__(self, name):
        return self.__Components[name]

    def __setitem__(self, name, component):
        if component is not None:
            if not isinstance(component, TopoDS_Shape):
                    raise TypeError('Component must be a TopoDS_Shape or None')
        self.__Components[name] = component

    def __delitem__(self, name):
        del self.__Components[name]

    def __iter__(self):
        return iter(self.__Components)

    def __len__(self):
        return len(self.__Components)
    
    def __str__(self):
        """Overloads print output to display the names of components in 
        the object instance"""
        output = str(self.keys()) 
        return output
        

    def AddComponent(self, component, name=None):
        """Adds a component to self
        Parameters
        ----------
        name - string
        component - TopoDS_Shape
        """
        if name is None:
            # set a default name:
            name = 'untitled_' + str(len(self))
        self.__setitem__(name, component)

    def RemoveComponent(self, name):
        """Removes a named component from self
        Parameters
        ----------
        name - string
        """
        self.__delitem__(name)

    def PrintComponents(self):
        """Lists the names of components in self"""
        outstring = 'Component names:'
        outstring += self.__str__()
        print(outstring)

    def TranslateComponents(self, vec):
        """Apply translation by vec to each component in self
        Parameters
        ----------
        vec - OCC.gp.gp_vec
            vector through which components will be translated
        """
        for name, component in self.items():
            self[name] = act.translate_topods_from_vector(component, vec)

    def RotateComponents(self, ax, deg):
        """Rotation of each component in self.__Components around ax by
        angle deg
        Parameters
        ----------
        ax - OCC.gp.gp_Ax1
            The axis of rotation
        deg - scalar
            Rotation in degrees
        """
        for name, component in self.items():
            self[name] = act.rotate(component, ax, deg)

    def TransformComponents_Nonuniformal(self, scaling, vec):
        """General scaling and translation of components in self
        (applies act.transform_nonuniformal)
        Parameters
        ----------
        scaling - list or array, length 3
            [x, y, z] scaling factors
       vec - List of x,y,z or gp_Vec
           the translation vector (default is [0,0,0])
        """
        for name, component in self.items():
            self[name] = act.transform_nonuniformal(component, scaling, vec)

    def Display(self, context, material=Graphic3d_NOM_ALUMINIUM):
        """Displays all components of this instance to input context
        Parameters
        ----------
        meterial - OCC.Graphic3d_NOM_* type
            The material for display: note some renderers do not allow this"""
        for name, component in self.items():
            ais = AIS_Shape(component)
            ais.SetMaterial(material)
            try:
                context.Context.Display(ais.GetHandle())
            except:
                context.DisplayShape(component)

    def MirrorComponents(self, plane='xz'):
        """Returns a mirrored version of this airconics shape
        Returns
        -------
        mirrored - AirconicsShape
            the mirrored shape
        Notes
        -----
        Due to problem with swig and deepcopy, the mirrored object is the
        base class 'AirconicsShape", not the original type. This is will
        remove other subclass-derived attributes and methods
    
        It is also expected that the remaining attributes and methods will not
        be required or meaningful after mirroring, however this behaviour
        may change in future versions
        """
        print("Note: MirrorComponents currently mirrors only the shape\n")
        print("components, other attributes will not be mirrored\n")
        mirrored = AirconicsShape()
        for name, component in self.items():
            mirrored[name] = act.mirror(component,
                                        plane,
                                        copy=True)
        return mirrored

    def Write(self, filename):
        """Writes the Components in this Airconics shape to filename using
        file format specified in extension of filename.
        Currently stl only (TODO: step, iges)

        Parameters
        ----------
        filename - string
        Returns
        -------
        status - list of int
            error status of the file output of EACH component
    
        Notes
        -----
        - File format is extracted from filename.
        - stl file write will prepend filename onto the Component name to be
        written to file (cannot write multiple files )
        """
        path, ext = os.path.splitext(filename)
        stl_writer = StlAPI_Writer()
        stl_ascii_format = False
        status = []
        for name, component in self.items():
            f = path + '_' + name + ext
            shape = component
            status.append(stl_writer.Write(shape, f, stl_ascii_format))
        return status


class AirconicsCollection(AirconicsBase):
    """Base class from which collections of parts defined by other Airconics
    classes will be stored. Allowable inputs are

    Notes
    -----
    Derived classes should call the AirconicsCollection init with
        super(DerivedClass, self).__init__(self, *args, **kwargs)
    """

    def __init__(self, parts={}, *args, **kwargs):
        # Set the components dictionary (default empty)
        self.__Parts = {}
        for name, part in parts.items():
            self.__setitem__(name, part)

        # Set all kwargs as attributes
        for key, value in kwargs.items():
            self.__setattr__(key, value)

    def __getitem__(self, name):
        return self.__Parts[name]

    def __setitem__(self, name, part):
        """Note no error checks done here: users responsible for content
        of this class"""
        self.__Parts[name] = part

    def __delitem__(self, name):
        del self.__Parts[name]

    def __iter__(self):
        return iter(self.__Parts)

    def __len__(self):
        return len(self.__Parts)
    
    def __str__(self):
        """Overloads print output to display the names of components in 
        the object instance"""
        output = str(self.keys())   # Note self.keys are self.__Parts.keys
        return output        

    def Write(self, filename):
        """Writes the Parts contained in this instance to file specified by
        filename. Currently one file is produced for each Part.

        Paramters
        ---------
        filename - string
            the BASE.ext name of the file e.g. 'airliner.stp'.
            Note the part name will be prepended to the base name of each
            output file

        Returns
        -------
        status - list
            The flattened list of error codes returned by writing each part

        Notes
        -----
        * Calls the .Write method belonging to each Part

        See Also: AirconicsBase
        """
        path, ext = os.path.splitext(filename)
        for name, part in self.items():
            f = path + '_' + name + ext
            part.Write(f)

    def Display(self, context, material=Graphic3d_NOM_ALUMINIUM):
        """Displays all Parts of the engine to input context
        Parameters
        ----------
        meterial - OCC.Graphic3d_NOM_* type
            The material for display: note some renderers do not allow this"""
        for name, component in self.items():
            component.Display(context, material)

    def AddPart(self, part, name=None):
        """Adds a component to self
        Parameters
        ----------
        name - string
        component - TopoDS_Shape
        """
        if name is None:
            # set a default name:
            name = 'untitled_' + str(len(self))
        self.__setitem__(name, part)
        
