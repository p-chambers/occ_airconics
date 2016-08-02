# -*- coding: utf-8 -*-
"""Base classes used by OCC_Airconics

Container classes (AirconicsBase, AirconicsShape, AirconicsCollection) which
behaves like a dictionary of sub component shapes/parts with some extended
functionality.


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
from OCC.gp import gp_Pnt


class AirconicsBase(MutableMapping, object):
    """Base container class from which other base classes are derived from.
    This is a an abstract base class and should not be used directly by users.

    Notes
    -----
    When properly defined in inherited functions, this class should behave like
    a dictionary.

    As this class inherits from MutableMapping, any class inherting from
    AirconicsBase must also define the abstract methods of Mutable mapping,
    i.e. __setitem__, __getitem__, __len__, __iter__, __delitem__
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

    @abstractmethod
    def Build(self, *args, **kwargs):
        pass


# class AirconicsContainer(MutableMapping):
#     """Simple container class which behaves as a dictionary, with all
#     attributes mapped to the values in self
#     """
#     def __init__(self, **kwargs):
#         self.__dict__.update(kwargs)

#     def __setitem__(self, name, component):
#         self.__dict__[name] = component

#     def __getitem__(self, name):
#         return self.__dict__[name]

#     def __delitem__(self, name):
#         del self.__dict__[key]

#     def __iter__(self):
#         return iter(self.__dict__)

#     def __len__(self):
#         return len(self.__dict__)

#     def __str__(self):
#         return str(self.__dict__)

#     __getattr__ = __getitem__
#     __setattr__ = __setitem__


class AirconicsShape(AirconicsBase):
    """Base class from which airconics parts will be made.

    AirconicsShapes represent a 'part' of an aircraft e.g. the engine, which
    consists of a group of logically shape 'components' but with no relative
    or relational contact information: class methods are intended to manipulate
    the part as a whole.

    This is intended as a base class, but can be used as a simple amorphic
    collection of shape components.

    :Example:
        >>> shape = Airconics()
        >>> shape['wing'] = wing  # OR
        >>> shape.AddComponent(wing, 'WingSurface')

    Parameters
    ----------
    components : dictionary of components to be added as attributes
        To add attributes directly. Values must be OCC.TopoDS.TopoDS_Shape

    construct_geometry : bool
        If true, Build method will be called on construction. Defaults to
        False for AirconicsShape, as the Build method only prints. Derived
        classes should pass construct_geometry=True if a Build method should
        be called on construction

    **kwargs : All other keyword arguments will be added as an attribute
        to the resulting class calling super(subclass, self).__init__

    Attributes
    ----------
    _Components : Airconics Container
        Mapping of name(string):component(TopoDS_Shape) pairs. Note that
        this should not be interacted with directly, and instead users should
        use assignment or the AddComponent method:

        :Example:
            >>> a = AirconicsShape()
            >>> a['name'] = shape
            >>> #OR: a.AddComponent('name', shape)

        This also supports mapping of attributes to parts, i.e:

        :Example:
            >>> a['name'] == a._Components.name   # returns True

    Notes
    -----
    Derived classes should call the AirconicsCollection init with
        super(DerivedClass, self).__init__(self, *args, **kwargs)

    See Also
    --------
    AirconicsCollection
    """

    def __init__(self, components={}, construct_geometry=False,
                 *args, **kwargs):
        # Set the components dictionary (default empty)
        self._Components = {}

        for name, component in components.items():
            self.__setitem__(name, component)

        # Set all kwargs as attributes - could move this to AirconicsBase
        for key, value in kwargs.items():
            self.__setattr__(key, value)

        self.construct_geometry = construct_geometry

        if self.construct_geometry:
            self.Build()
        else:
            print("Skipping geometry construction for {}".format(
                type(self).__name__))

    def __getitem__(self, name):
        return self._Components[name]

    def __setitem__(self, name, component):
        if component is not None:
            if not isinstance(component, TopoDS_Shape):
                    raise TypeError('Component must be a TopoDS_Shape or None')
        self._Components[name] = component

    def __delitem__(self, name):
        del self._Components[name]

    def __iter__(self):
        return iter(self._Components)

    def __len__(self):
        return len(self._Components)

    def __str__(self):
        """Overloads print output to display the names of components in
        the object instance"""
        output = str(self.keys())
        return output

    def Build(self):
        """Does nothing for AirconicsShape.

        This method allows AirconicsShape to be instantiated alone, as Build
        is called in the __init__. 'Build' Should be redefined by all derived
        classes

        Notes
        -----
        * If Class.Build is not redefined in a derived class, confusion may
        arise as no geometry will result from passing construct_geometry=True
        """
        print("Attempting to construct {} geometry...".format(
            type(self).__name__))

    def AddComponent(self, component, name=None):
        """Adds a component to self

        Parameters
        ----------
        component : TopoDS_Shape

        name : string
        """
        if name is None:
            # set a default name:
            name = 'untitled_' + str(len(self))
        self.__setitem__(name, component)

    def RemoveComponent(self, name):
        """Removes a named component from self

        Parameters
        ----------
        name : string
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
        vec : OCC.gp.gp_vec
            vector through which components will be translated
        """
        for name, component in self.items():
            self[name] = act.translate_topods_from_vector(component, vec)

    def RotateComponents(self, ax, deg):
        """Rotation of each component in self._Components around ax by
        angle deg

        Parameters
        ----------
        ax : OCC.gp.gp_Ax1
            The axis of rotation

        deg : scalar
            Rotation in degrees
        """
        for name, component in self.items():
            self[name] = act.rotate(component, ax, deg)

    def ScaleComponents_Uniformal(self, factor, origin=gp_Pnt(0, 0, 0)):
        """General scaling and translation of components in self
        (applies act.transform_nonuniformal)

        Parameters
        ----------
        origin : gp_Pnt
            The origin of the scaling operation

        factor : scalar
            The scaling factor to apply in x,y,z
        """
        for name, component in self.items():
            self[name] = act.scale_uniformal(component, origin, factor)

    def TransformComponents_Nonuniformal(self, scaling, vec):
        """General scaling and translation of components in self
        (applies act.transform_nonuniformal)

        Parameters
        ----------
        scaling : list or array, length 3
            [x, y, z] scaling factors

        vec : List of x,y,z or gp_Vec
           the translation vector (default is [0,0,0])
        """
        for name, component in self.items():
            self[name] = act.transform_nonuniformal(component, scaling, vec)

    def Display(self, context, material=Graphic3d_NOM_ALUMINIUM, color=None):
        """Displays all components of this instance to input context

        Parameters
        ----------
        context : OCC.Display.OCCViewer.Viewer3d or WebRenderer
            The display context - should have a Display or DisplayShape method

        meterial : OCC.Graphic3d_NOM_* type (default=ALUMINIUM)
            The material for display: note some renderers do not allow this

        color : string
            The color for all components in this shape
        """
        for name, component in self.items():
            ais = AIS_Shape(component)
            ais.SetMaterial(material)
            if color:
                try:
                    from OCC.Display.OCCViewer import get_color_from_name
                    color = get_color_from_name(color)
                except:
                    pass
                ais.SetColor(color)
            try:
                context.Context.Display(ais.GetHandle())
            except:
                context.DisplayShape(component)

    def MirrorComponents(self, plane='xz', axe2=None):
        """Returns a mirrored version of this airconics shape

        Parameters
        ----------
        plane : string (default='xz')
            The plane in which to mirror components

        axe2 : OCC.gp.gp_Ax2
            The axes through which to mirror (overwrites input 'plane')

        Returns
        -------
        mirrored : AirconicsShape
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
        print("Note: MirrorComponents currently mirrors only the shape")
        print("components, other attributes will not be mirrored\n")
        mirrored = AirconicsShape()
        for name, component in self.items():
            mirrored[name] = act.mirror(component,
                                        plane,
                                        copy=True)
        return mirrored

    def Write(self, filename, single_export=True):
        """Writes the Components in this Airconics shape to filename using
        file format specified in extension of filename.
        Currently stl only (TODO: step, iges)

        Parameters
        ----------
        filename : string
            the BASE.ext name of the file e.g. 'airliner.stp'.
            Note the Component name will be prepended to the base name of each
            output file
        
        single_export : bool
            Writes a single output file if true, otherwise writes one file
            per component

        Returns
        -------
        status : list of int
            error status of the file output of EACH component

        Notes
        -----
        File format is extracted from filename.

        stl file write will prepend filename onto the Component name to be
        written to file (cannot write multiple files )
        """
        path, ext = os.path.splitext(filename)
        status = []
        if ext == '.stl':
            stl_ascii_format = False
            if single_export:
                stl_writer = StlAPI_Writer()
                for name, component in self.items():
                    shape = component
                    status.append(stl_writer.Write(shape, filename,
                                                   stl_ascii_format))
            else:
                for name, component in self.items():
                    stl_writer = StlAPI_Writer()
                    f = path + '_' + name + ext
                    shape = component
                    status.append(stl_writer.Write(shape, f, stl_ascii_format))

        elif ext in ['.stp', '.step']:
            if single_export:
                status.append(act.export_STEPFile(self.values(), filename))
            else:
                for name, component in self.items():
                    f = path + '_' + name + ext
                    status.append(act.export_STEPFile([component], f))
        else:
            raise ValueError('Unexpected file extension {}'.format(ext))

        return status


class AirconicsCollection(AirconicsBase):
    """Base class from which collections of parts defined by other Airconics
    classes will be stored.

    AirconicsCollection represents a collection of 'parts'
    (i.e. AirconicsShapes') which are logically grouped. For example, an
    aircraft comprised of multiple parts (engine, lifting surfaces, fuselage)
    all of which may contain sub 'components' and are therefore instances of
    AirconicsShapes'

    Parameters
    ----------
    parts : dictionary
        (name: part) pairs, where name is a string for accessing the part,
        and 'part' is an AirconicsShape derived class e.g. Fuselage,
        LiftingSurface or Engine instance

    Attributes
    ----------
    _Parts : Airconics Container
        Mapping of name(string):component(AirconicsShape) pairs. Note that
        this should not be interacted with directly, and instead users should
        use assignment or the AddPart method:

        :Example:
            >>> a = AirconicsCollection()
            >>> a['name'] = part
            >>> #OR: a.AddPart('name', part)

        This also supports mapping of attributes to parts, i.e:

        :Example:
            >>> a['name'] == a._Parts.name   # returns True

    Notes
    -----
    Derived classes should call the AirconicsCollection init with
        super(DerivedClass, self).__init__(self, *args, **kwargs)

    See Also
    --------
    AirconicsShape
    """

    def __init__(self, parts={}, construct_geometry=False, *args, **kwargs):
        # Set the components dictionary (default empty)
        self._Parts = {}
        for name, part in parts.items():
            self.__setitem__(name, part)

        # Set all kwargs as attributes
        for key, value in kwargs.items():
            self.__setattr__(key, value)

        self.construct_geometry = construct_geometry

        if self.construct_geometry:
            self.Build()
        else:
            print("Skipping geometry construction for {}".format(
                type(self).__name__))

    def __getitem__(self, name):
        return self._Parts[name]

    def __setitem__(self, name, part):
        """Note no error checks done here: users responsible for content
        of this class"""
        self._Parts[name] = part

    def __delitem__(self, name):
        del self._Parts[name]

    def __iter__(self):
        return iter(self._Parts)

    def __len__(self):
        return len(self._Parts)

    def __str__(self):
        """Overloads print output to display the names of components in
        the object instance"""
        output = str(self.keys())   # Note self.keys are self._Parts.keys
        return output

    def Write(self, filename, single_export=True):
        """Writes the Parts contained in this instance to file specified by
        filename.

        One file is produced, unless single_export is False when one file
        is written for each Part.

        Parameters
        ---------
        filename : string
            the BASE.ext name of the file e.g. 'airliner.stp'.
            Note the part name will be prepended to the base name of each
            output file

        single_export : bool
            returns a single output file if true, otherwise writes one file
            per part

        Returns
        -------
        status : list
            The flattened list of error codes returned by writing each part

        Notes
        -----
        * Calls the .Write method belonging to each Part

        See Also
        --------
        AirconicsBase
        """
        path, ext = os.path.splitext(filename)

        status = []

        if ext == '.stl':
            stl_ascii_format = False

            if single_export:
                stl_writer = StlAPI_Writer()
                for partname, part in self.items():
                    for name, component in part.items():
                        status.append(stl_writer.Write(component, filename,
                                                       stl_ascii_format))
            else:
                for partname, part in self.items():
                    f = path + '_' + name + ext
                    status.append(path.Write(f, single_export=True))

        elif ext in ['.stp', '.step']:
            if single_export:
                shapes = []
                for partname, part in self.items():
                    shapes.extend(part.values())
                status.append(act.export_STEPFile(shapes, filename))
            else:
                for name, part in self.items():
                    f = path + '_' + name + ext
                    # Writes one file per part
                    status.append(part.Write(f, single_export=True))

        return status

    def Build(self):
        """Does nothing for AirconicsCollection.

        This method allows AirconicsColection to be instantiated alone, as
        Build is called in the __init__. 'Build' Should be redefined by all
        derived classes.

        Notes
        -----
        * If Class.Build is not redefined in a derived class, confusion may
        arise as no geometry will result from passing construct_geometry=True
        """
        print("Attempting to construct {} geometry...".format(
            type(self).__name__))

    def Display(self, context, material=Graphic3d_NOM_ALUMINIUM, color=None):
        """Displays all Parts of the engine to input context

        Parameters
        ----------
        context : OCC.Display.OCCViewer.Viewer3d or WebRenderer
            The display context - should have a Display or DisplayShape method

        meterial : OCC.Graphic3d_NOM_* type
            The material for display: note some renderers do not allow this
        """
        for name, component in self.items():
            try:
                component.Display(context, material, color)
            except AttributeError:
                # We are probably dealing with a core pythonocc shape:
                try:
                    context.DisplayShape(component)
                except:
                    print("Could not display shape type {}: skipping".format(
                        type(component)))

    def AddPart(self, part, name=None):
        """Adds a component to self

        Parameters
        ----------
        part : TopoDS_Shape

        name : string
        """
        if name is None:
            # set a default name:
            name = 'untitled_' + str(len(self))
        self.__setitem__(name, part)
