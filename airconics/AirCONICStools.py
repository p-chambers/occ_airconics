# -*- coding: utf-8 -*-
"""
Various geometry operations of geometric pythonocc primitives for OCC_AirCONICS

Created on Fri Dec  4 11:58:52 2015

@author: pchambers
"""
# Geometry Manipulation libraries:
import OCC.Bnd
from OCC.AIS import AIS_WireFrame, AIS_Shape
from OCC.Geom import Geom_BezierCurve
from OCC.GeomAPI import (GeomAPI_PointsToBSpline, GeomAPI_IntCS,
                         GeomAPI_Interpolate)
from OCC.BRepBndLib import brepbndlib_Add
from OCC.TColgp import (TColgp_Array1OfPnt, TColgp_HArray1OfPnt,
                        TColgp_Array1OfVec)
from OCC.TColStd import TColStd_HArray1OfBoolean
from OCC.BRepOffsetAPI import  (BRepOffsetAPI_ThruSections,
                                BRepOffsetAPI_MakePipeShell)
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeEdge,
                                BRepBuilderAPI_Transform,
                                BRepBuilderAPI_MakeFace,
                                BRepBuilderAPI_GTransform,
                                BRepBuilderAPI_MakeVertex)
from OCC.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone,
                             BRepPrimAPI_MakeHalfSpace,
                             BRepPrimAPI_MakeSphere)
from OCC.BRepAlgoAPI import BRepAlgoAPI_Section, BRepAlgoAPI_Cut
from OCC.gp import (gp_Trsf, gp_Ax2, gp_Pnt, gp_Dir, gp_Vec, gp_Pln,
                    gp_GTrsf, gp_Mat, gp_XYZ)
from OCC.GeomAbs import GeomAbs_C2
from OCC.TopoDS import *
from OCC.TopAbs import *
from OCC.TopExp import TopExp_Explorer
from OCC.GC import GC_MakeCircle, GC_MakeSegment
from OCC.Approx import Approx_ChordLength
from OCC.GCPnts import GCPnts_UniformAbscissa
from OCC.GeomAdaptor import GeomAdaptor_Curve, GeomAdaptor_HCurve
from OCC.GeomPlate import (GeomPlate_CurveConstraint,
                           GeomPlate_BuildPlateSurface,
                           GeomPlate_MakeApprox)
from OCC.BRepAdaptor import BRepAdaptor_Curve
from OCC.BRepFeat import BRepFeat_SplitShape
from OCC.TopTools import TopTools_ListIteratorOfListOfShape
from OCC.BRepProj import BRepProj_Projection

# FileIO libraries:
from OCC.STEPCAFControl import STEPCAFControl_Writer
from OCC.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Interface import Interface_Static_SetCVal
from OCC.IFSelect import IFSelect_RetDone
from OCC.TDF import TDF_LabelSequence
from OCC.TCollection import TCollection_ExtendedString
from OCC.TDocStd import Handle_TDocStd_Document
from OCC.XCAFApp import XCAFApp_Application
from OCC.XCAFDoc import (XCAFDoc_DocumentTool_ShapeTool,
                         XCAFDoc_DocumentTool_ColorTool,
                         XCAFDoc_DocumentTool_LayerTool,
                         XCAFDoc_DocumentTool_MaterialTool)

# Standard Python libraries
import numpy as np


class assert_isdone(object):
    '''
    raises an assertion error when IsDone() returns false, with the error
    specified in error_statement
    -> this is from the pythonocc-utils utility-may not use it?
    '''
    def __init__(self, to_check, error_statement):
        self.to_check = to_check
        self.error_statement = error_statement

    def __enter__(self, ):
        if self.to_check.IsDone():
            pass
        else:
            raise AssertionError(self.error_statement)

    def __exit__(self, type, value, traceback):
        pass


# TODO: Add TE function (required for creating tip face)
#def AddTEtoOpenAirfoil(Airfoil):
#    """If the airfoil curve given as an argument is open at the trailing edge,
#    adds a line between the ends of the curve and joins this with the rest
#    of the curve. """
#    assert(hasattr(Airfoil, 'Curve')), 'Input object does not have a Curve atribute'
#
#    handle = Airfoil.Curve.GetObject()
#    if not handle.IsClosed():
#        try:
#            EP = handle.EndPoint()
#            SP = handle.StartPoint()
#            Closure = gp_Lin(Ep, SP)
#            shapeconstruct
#    else:
#        print("Curve is already closed")
#
#    assert(handle.IsClosed()), "Failed to Add Trailing Edge"
#
#    return None


def ObjectsExtents(breps, tol=1e-6, as_vec=False):
    """Compute the extents in the X, Y and Z direction (in the current
    coordinate system) of the objects listed in the argument.

    Parameters
    ----------
    breps : list of TopoDS_Shape
        The shapes to be added for bounding box calculation
    
    tol : float (default=1e-6)
        Tolerance for bounding box calculation
    
    as_vec : bool (default=False)
        If true, returns minimum and maximum points as tuple of gp_Vec

    Returns
    -------
    xmin, ymin, zmin, xmax, ymax, zmax : scalar
        the min and max points of bbox (returned if as_vec=False)
    
    ( gp_Vec(xmin, ymin, zmin), gp_Vec(xmax, ymax, zmax) ) : tuple of gp_Vec
        the min and max points of bbox (returned in as_vec=True)

    Notes
    -----
    Due to the underlying OCC.Bnd.Bnd_Box functions, the bounding box is 
    calculated via triangulation of the shapes to avoid inclusion of the 
    control points of NURBS curves in bounding box calculation
    """

    bbox = OCC.Bnd.Bnd_Box()
    bbox.SetGap(tol)

    try:
        for shape in breps:
            brepbndlib_Add(shape, bbox, True)
    except TypeError:
        # Assume not iterable:
        brepbndlib_Add(breps, bbox, True)

    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    if as_vec is False:
        return xmin, ymin, zmin, xmax, ymax, zmax
    else:
        return gp_Vec(xmin, ymin, zmin), gp_Vec(xmax, ymax, zmax)


def BBox_FromExtents(xmin, ymin, zmin, xmax, ymax, zmax):
    """Generates the Wire Edges defining the Bounding Box defined in the input
    arguments: Can be used to display the bounding box"""
    s = BRepPrimAPI_MakeBox(gp_Pnt(xmin, ymin, zmin),
                            gp_Pnt(xmax, ymax, zmax)).Shape()
    ais_bbox = AIS_Shape(s)
    ais_bbox.SetDisplayMode(AIS_WireFrame)
    return ais_bbox.GetHandle()


def point_array_to_TColgp_PntArrayType(array, _type=TColgp_Array1OfPnt):
    """Function to return curve from numpy array

    Parameters
    ----------
    array : array (Npts x 3) or list
        Array of xyz points for which to fit a bspline

    _type : type of TColgp array
        Tested inputs are,
            - TColgp_Array1OfPnt
            - TColgp_HArray1OfPnt
        See Notes for more information

    Returns
    -------
    pt_arr : TCOLgp_Array1OfPnt
        OCC type array of points

    Notes
    -----
    USe TColgp_Harray when interpolating a curve from points with the
    GeomAPI_Interpolate. Use TColgp_Array when interpolating a curve
    from points with the GeomAPI_PointsToBspline
    """
    try:
        dims = np.shape(array)
        # Attempt to tranpose if x, y, z points span columns (should span rows)
        if dims[0] == 3:
            array = array.T
        elif dims[1] != 3:
            raise ValueError("Array must have dimension Npnts x 3 (x, y, z)")
        N = np.shape(array)[0]
        pt_arr = _type(1, N)
        for i, pt in enumerate(array):
            pt_arr.SetValue(i+1, gp_Pnt(*pt))
    except:
        # Input pnts are likely to be a list of gp_Pnt:
        N = len(array)
        pt_arr = _type(1, N)
        for i, pt in enumerate(array):
            pt_arr.SetValue(i+1, pt)
    return pt_arr


def points_to_bspline(pnts, deg=3, periodic=False, tangents=None,
                      scale=False, continuity=GeomAbs_C2):
    """
    Points to bspline: originally from pythonocc-utils, changed to allow numpy
    arrays as input

    Paramters
    ---------
    pnts : list or numpy array
        array of x, y, z points

    deg : integer
        degree of the fitted bspline

    periodic : Bool (default=False)
        If true, OCC.GeomAPI_Interpolate will be used instead of the
        GeomAPI_PointsToBspline. Curve tangent vectors can then be
        enforced at the interpolation pnts

    tangents : array (default=None)
        list of [x,y,z] tangent vectors to be specificied at points:
        if only 2 tangents are specified, these will be enforced at the
        start and end points, otherwise tangents should have the same length
        as pnts and will be enforced at each point.

    Scale : Bool (default=False)
        Will scale the tangents (gives a smoother Periodic curve if False)

    continuity : OCC.GeomAbs.GeomAbs_XX type (default C2)
        The order of continuity (C^0, C^1, C^2, G^0, ....)    
    
    Returns
    -------
    crv : OCC.Geom.BSplineCurve
    
    Notes
    -----

    """
    if not periodic and (tangents is None):
        _type = TColgp_Array1OfPnt
        pnts = point_array_to_TColgp_PntArrayType(pnts, _type)
        # Fit the curve to the point array
        deg_min = deg
        deg_max = deg
        crv = GeomAPI_PointsToBSpline(pnts, deg_min, deg_max, continuity).Curve()
    else:
        _type = TColgp_HArray1OfPnt
        pnts = point_array_to_TColgp_PntArrayType(pnts, _type)
        tol = 0.001
        interp = GeomAPI_Interpolate(pnts.GetHandle(), periodic, tol)
        if tangents is not None:
            N = tangents.shape[0]
            if N == 2:
                interp.Load(gp_Vec(*tangents[0,:]), gp_Vec(*tangents[1,:]),
                            scale)
            else:
                tan_array = TColgp_Array1OfVec(1, N)
                for i in xrange(1, N+1):
                    tan_array.SetValue(i, gp_Vec(*tangents[i-1,:]))
                tan_flags = TColStd_HArray1OfBoolean(1, N)
                tan_flags.Init(True)   #Set all true i.e. enforce all tangents
                interp.Load(tan_array, tan_flags.GetHandle(), scale)
        interp.Perform()
        crv = interp.Curve()
    return crv


def points_to_BezierCurve(pnts):
    """
    Creates a Bezier curve from an array of points.

    Parameters
    ----------
    pnts : array or list
        x, y, z for an array of points. Allowable inputs are numpy arrays
        (with dimensions (Npoints x 3)), python list with elements [xi, yi, zi]
        or list of OCC.gp.gp_Pnt objects

    Returns
    -------
    crv : OCC.Geom.Geom_BezierCurve
    """
    pnts = point_array_to_TColgp_PntArrayType(pnts, TColgp_Array1OfPnt)
    # Fit the curve to the point array
    crv = Geom_BezierCurve(pnts)
    return crv


def scale_uniformal(brep, pnt, factor, copy=False):
    '''
    translate a brep over a vector : from pythonocc-utils

    Paramters
    ---------
    brep : TopoDS_Shape
        the TopoDS_Shape to scale

    pnt : gp_Pnt
        Origin of scaling

    factor : scalar
        Scaling factor

    copy : bool
        copies to brep if True
    '''
    trns = gp_Trsf()
    trns.SetScale(pnt, factor)
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def transform_nonuniformal(brep, factors, vec=[0, 0, 0], copy=False):
    """Nonuniformly scale brep with respect to pnt by the x y z scaling factors
    provided in 'factors', and translate by vector 'vec'

    Parameters
    ----------
    factors : List of factors [Fx, Fy, Fz]
        Scaling factors with respect to origin (0,0,0)

    vec : List of x,y,z or gp_Vec
        the translation vector (default is [0,0,0])

    Notes
    -----
    * Only tested on 3d shapes
    * Assumes factors are define with respect to the origin (0,0,0)
    """
    assert(len(factors) == 3),\
        ("factors should have [Fx, Fy, Fz] scaling factors: Found length ",
         len(factors))
    M = np.diag(factors).flatten()
    trns_M = gp_Mat(*M)

    try:
        V = gp_XYZ(*vec)
    except NotImplementedError:
        V = gp_XYZ(vec.X(), vec.Y(), vec.Z())
    trns = gp_GTrsf(trns_M, V)

    brep_trns = BRepBuilderAPI_GTransform(brep, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def FilletFaceCorners(face, radius):
    """Fillets the corners of the input face
    
    Parameters
    ----------
    face : TopoDS_Face
        
    radius : the Fillet radius
    
    Returns
    -------
    """
    vert_explorer = TopExp_Explorer(face, TopAbs_VERTEX)
    from OCC.BRepFilletAPI import BRepFilletAPI_MakeFillet2d
    fillet = BRepFilletAPI_MakeFillet2d(face)
    while vert_explorer.More():
        vertex = topods_Vertex(vert_explorer.Current())
        fillet.AddFillet(vertex, radius)
        # Note: Needed two next statements here as faces have a vertex on
        # either side
        vert_explorer.Next()
        vert_explorer.Next()

    fillet.Build()
    face = fillet.Shape()
    return face


def ExtrudeFace(face, vec=gp_Vec(1, 0, 0)):
    """Extrudes a face by input vector
    
    Parameters
    ----------
    face : TopoDS_Face
    
    vec : OCC.gp.gp_Vec
        The offset vector to extrude through

    Returns
    -------
    shape : TopoDS_Shape
        The extruded shape
    
    Notes
    -----
    Uses BRepBuilderAPI_MakePrism
    """
    from OCC.BRepPrimAPI import BRepPrimAPI_MakePrism
    builder = BRepPrimAPI_MakePrism(face, vec)
    builder.Build()
    return builder.Shape()


def SplitShapeFromProjection(shape, wire, direction, return_section=True):
    """Splits shape by the projection of wire onto its face
    
    Parameters
    ----------
    shape : TopoDS_Shape    
        the brep to subtract from 

    wire : TopoDS_Wire
        the tool to use for projection and splitting 

    direction: OCC.gp.gp_Dir
        the direction to project the wire

    return_section : bool
        returns the split shape 

    Returns
    -------
    newshape : TopoDS_Shape
        input shape with wire subtracted
    
    section : the shape which was substracted
        (returned only if return_section is true)
    
    Notes
    -----
    Currently assumes splits the first face only
    """
#    get the face from the shape
    exp = TopExp_Explorer(shape, TopAbs_FACE)
    face = topods_Face(exp.Current())

#    Perform the projection
    proj = BRepProj_Projection(wire, face, direction)
    wire = proj.Current()

    splitter = BRepFeat_SplitShape(face)
    splitter.Add(wire, face)
    splitter.Build()
    
    section_list = splitter.DirectLeft()
    iterator = TopTools_ListIteratorOfListOfShape(section_list)
    section = iterator.Value()    # assume here that only 1 section is produced
    
    mod_list = splitter.Modified(face)
    iterator = TopTools_ListIteratorOfListOfShape(mod_list)
    newshape = iterator.Value()
    
    if return_section:
        return shape, wire
    else:
        return newshape


def coslin(TransitionPoint, NCosPoints=24, NLinPoints=24):
    """Creates a series of abscissas with cosine spacing from 0 to a
    TransitionPoint and a linear spacing thereafter, up to 1. The
    TransitionPoint corresponds to pi. Distribution suitable for airfoils
    defined by points. TransitionPoint must be in the range [0,1].

    Parameters
    ----------
    TransitionPoint : scalar
        Point to transition from cosine to linear distribution in range (0, 1)
    
    NCosPoints : int
        Number of points to space by cosine law between 0 and TransitionPoint
    
    NLinPoints : int
        Number of points to space by linear law between TransitionPoint and 1

    Returns
    -------
    Abscissa : numpy array
        The generated abscissa
    
    NCosPoints : int
        Number of cosine points used (same as input)
    """
    angles = np.linspace(0, np.pi/2., NCosPoints)
    cos_pts = TransitionPoint * (1.-np.cos(angles))
    lin_pts = np.linspace(TransitionPoint, 1., NLinPoints+1)
    # Combine points and Remove first linear point (already in cos_pts):
    Abscissa = np.hstack((cos_pts, lin_pts[1:]))
    return Abscissa, NCosPoints


def export_STEPFile(shapes, filename):
    """Exports a .stp file containing the input shapes

    Parameters
    ----------
    shapes : list of TopoDS_Shape
        Shapes to write to file

    filename : string
        The output filename
    """
    # initialize the STEP exporter
    step_writer = STEPControl_Writer()
#    Interface_Static_SetCVal("write.step.schema", "AP214") # Use default?

    # transfer shapes
    for shape in shapes:
        step_writer.Transfer(shape, STEPControl_AsIs)

    status = step_writer.Write(filename)

    assert(status == IFSelect_RetDone)
    return status

#
#def export_STLFile(AC_Shapes, filename):
#    """Writes a component stl file for each shape in input AirCONICS shapes"""
#    try:
#        for shape in AC_Shapes:
#            status = shape.WriteComponents(filename)
#    except:
#        # Assume error was raised as AC_Shapes contains only one shape
#        status = shape.WriteComponents(filename)[0]
#    return status


def export_STEPFile_Airconics(AirconicsShapes, filename):
    """ Writes a Step file with names defined in the AirconicsShapes. This
    function is not fully tested and should not yet be used.

    Notes
    -----
    Work in progress
    """
    print("This function is a work in progress. For now, use export_STEPFile")
    # create an handle to a document
    h_doc = Handle_TDocStd_Document()

    # Create the application
    app = XCAFApp_Application.GetApplication().GetObject()
    app.NewDocument(TCollection_ExtendedString("MDTV-CAF"), h_doc)

    # Get root assembly
    doc = h_doc.GetObject()
    shape_tool = XCAFDoc_DocumentTool_ShapeTool(doc.Main()).GetObject()
#    l_colors = XCAFDoc_DocumentTool_ColorTool(doc.Main())
#    l_layers = XCAFDoc_DocumentTool_LayerTool(doc.Main())
#    l_materials = XCAFDoc_DocumentTool_MaterialTool(doc.Main())

    step_writer = STEPCAFControl_Writer()
    step_writer.SetColorMode(True)
    step_writer.SetLayerMode(True)
    step_writer.SetNameMode(True)
#    step_writer.SetMaterialMode(True)

    for ACshape in AirconicsShapes:
        for comp in ACshape.Components:
            print("Writing {} to {}".format(comp, filename))
            lbl = shape_tool.AddShape(ACshape.Components[comp])
            name = TCollection_ExtendedString(comp)
#            tdn = TDataStd_Name()
#            tdn.Set(lbl, name)
            step_writer.Transfer(lbl, STEPControl_AsIs)

    status = step_writer.Write(filename)
    assert(status == IFSelect_RetDone)
    return status


#def import_STEPFile(shapes, filename):
#    # TODO: initialize the STEP importer
#    step_writer = STEPControl_Reader()
#
#    status = step_reader.ReadFile(filename)
#
#    assert(status == IFSelect_RetDone)
#    return status


def AddSurfaceLoft(objs, continuity=GeomAbs_C2, check_compatibility=True,
                   solid=True, first_vertex=None, last_vertex=None,
                   max_degree=8, close_sections=True):
    """Create a lift surface through curve objects

    Parameters
    ----------
    objs : list of python classes
        Each obj is expected to have an obj.Curve attribute :
        see airconics.primitives.airfoil class
    
    continuity : OCC.GeomAbs.GeomAbs_XX type (default C2)
        The order of continuity (C^0, C^1, C^2, G^0, ....)
    
    check_compatibility : bool (default=True)
        Adds a surface compatibility check to the builder

    solid : bool (default=True)
        Creates a solid object from the loft if True
    
    first_vertex : TopoDS_Vertex (optional, default=None)
        The starting vertex of the surface to add to the 'ThruSections'
        algorithm

    last_vertex : TopoDS_Vertex (optional, default=None)
        The end vertex of the surface to add to the 'ThruSections'
        algorithm    

    max_degree : int (default=8)
        The order of the fitted NURBS surface

    close_sections : bool (default=True):
        Connects the start and end point of the loft rib curves if true. This
        has the same effect as adding an airfoil trailing edge.

    Returns
    -------
    shape : TopoDS_Shape
        The generated loft surface
    
    Notes
    -----
    Uses OCC.BRepOffsetAPI.BRepOffsetAPI_ThruSections. This function is
    ORDER DEPENDANT, i.e. add elements in the order through which they should
    be lofted
    """
    assert(len(objs) >= 2), 'Loft Failed: Less than two input curves'
    # Note: This is to give a smooth loft.
    ruled = False
    pres3d = 1e-6
    args = [solid, ruled, pres3d]    # args (in order) for ThruSections
    generator = BRepOffsetAPI_ThruSections(*args)
    generator.SetMaxDegree(max_degree)
#    from OCC.GeomAbs import GeomAbs_G1
    generator.SetParType(Approx_ChordLength)
    if first_vertex:
        generator.AddVertex(first_vertex)
    for obj in objs:
        try:
            # Check if this is an airconics object with a GeomBspline handle
            # as its 'Curve' attribute
            obj = obj.Curve
#            edge = [make_edge(obj)]
        except:
            # Assume the object is already a geombspline handle
            pass
#            try:
#                # If working with an airconics object, the OCC curve is stored
#                # in obj.Curve:
        edges = [make_edge(obj)]

        if close_sections:
            crv = obj.GetObject()
            if crv.IsClosed() is False:
                # Add Finite TE edge
                TE = make_edge(crv.EndPoint(), crv.StartPoint())
                edges.append(TE)

        generator.AddWire(BRepBuilderAPI_MakeWire(*edges).Wire())
#        else:
#            generator
    if last_vertex:
        generator.AddVertex(last_vertex)

    generator.CheckCompatibility(check_compatibility)
    generator.SetContinuity(continuity)
    generator.Build()
    with assert_isdone(generator, 'failed lofting'):
        return generator.Shape()


def Generate_InterpFunction(Values, EpsArray=None, uniform=True):
    """Generates a lookup interpolation function.

    Given an array of spanwise coordinates epsilon along a curvilinear
    leading-edge attached coordinate system, and a set of values describing
    e.g. Chord, Sweep at each station, generate and return a function
    f(epsilon) which will give the interpolated value.

    Parameters
    ----------
    Values : array of float
        Values of e.g. chordlength, sweep at each spanwise location in EpsArray

    EpsArray : array of float
        Distribution of spanwise coordinates at which the Values are known

    uniform : bool
        If True, assumes that Values corresponds to uniformly distribution
        epsilon locations along the lifting surface span

    Returns
    -------
    f : function
        the function which returns the interpolated epsilon
    """
    if uniform:
        EpsArray = np.linspace(0, 1, len(Values))

    def f(Epsilon):
        return np.interp(Epsilon, EpsArray, Values)

    return f


def translate_topods_from_vector(brep_or_iterable, vec, copy=False):
    '''
    Function Originally from pythonocc-utils, modified to work on objects

    translates a brep over a vector

    Parameters
    ----------
    brep : the Topo_DS to translate

    vec : the vector defining the translation

    copy : copies to brep if True
    '''
    trns = gp_Trsf()
    trns.SetTranslation(vec)
    if issubclass(brep_or_iterable.__class__, TopoDS_Shape):
        brep_trns = BRepBuilderAPI_Transform(brep_or_iterable, trns, copy)
        brep_trns.Build()
        return brep_trns.Shape()
    else:
        return [translate_topods_from_vector(brep_or_iterable, vec, copy) for i in brep_or_iterable]


def Uniform_Points_on_Curve(curve, NPoints):
    """Returns a list of uniformly spaced points on a curve

    Parameters
    ----------
    crv : OCC.Geom curve type

    NPoints : int
        number of sampling points along the curve"""
    try:
        adapt = GeomAdaptor_Curve(curve)
    except:
        # Allow the algorithm to deal with TopoDS_Edge and Wire shapes:
        adapt = BRepAdaptor_Curve(curve)
    absc = GCPnts_UniformAbscissa(adapt, NPoints)
    return [adapt.Value(absc.Parameter(i)) for i in xrange(1, NPoints+1)]

def rotate(brep, axe, degree, copy=False):
    """Rotates the brep
    
    Originally from pythonocc-utils : might add dependency on this?

    Parameters
    ----------
    brep : shape to rotate
    
    axe : axis of rotation
    
    degree : Number of degrees to rotate through
    
    copy : bool (default=False)
    
    Returns
    -------
    BRepBuilderAPI_Transform.Shape : Shape handle
        The handle to the rotated shape
    """
    trns = gp_Trsf()
    trns.SetRotation(axe, np.radians(degree))
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def mirror(brep, plane='xz', axe2=None, copy=False):
    """Originally from pythonocc-utils : might add dependency on this?
    Mirrors object

    Parameters
    ----------
    brep : OCC.TopoDS.TopoDS_Shape
        The shape to mirror

    plane : string (default = 'xz')
        The name of the plane in which to mirror objects. Acceptable inputs are
        any of 'xy', 'yx' , 'zy', 'yz', 'yz', 'zy'. Overwritten if axe2 is
        defined.

    axe2 : OCC.gp.gp_Ax2
        The axes through which to mirror (overwrites input 'plane')

    copy : bool
        
    Returns
    -------
    BRepBuilderAPI_Transform.Shape : TopoDS_Shape
        The reflected shape
    
    Notes
    -----
    Pchambers: Added a functionality here to specify a plane using a string so
    that users could avoid interacting with core occ objects"""
    if axe2:
        plane = None
    else:
        Orig = gp_Pnt(0., 0., 0.)        
        if plane in ['xz', 'zx']:
            ydir = gp_Dir(0, 1, 0)
            axe2 = gp_Ax2(Orig, ydir)
        elif plane in ['yz', 'zy']:
            xdir = gp_Dir(1, 0, 0)
            axe2 = gp_Ax2(Orig, xdir)
        elif plane in ['xy', 'yx']:
            zdir = gp_Dir(0, 0, 1)
            axe2 = gp_Ax2(Orig, zdir)
        else:
            raise(ValueError, "Unknown mirror plane string,", plane)
    trns = gp_Trsf()
    trns.SetMirror(axe2)
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    return brep_trns.Shape()


# TODO: Curve fairing functions
#def batten_curve(pt1, pt2, height, slope, angle1, angle2):
#    fc = FairCurve_MinimalVariation(pt1, pt2, height, slope)
#    fc.SetConstraintOrder1(2)
#    fc.SetConstraintOrder2(2)
#    fc.SetAngle1(angle1)
#    fc.SetAngle2(angle2)
#    fc.SetHeight(height)
#    fc.SetSlope(slope)
#    fc.SetFreeSliding(True)
#    print(fc.DumpToString())
#    status = fc.Compute()
#    print(error_code(status[0]), error_code(status[1]))
#    return fc.Curve()
#
#
#def faircurve(event=None):
#    pt1 = gp_Pnt2d(0., 0.)
#    pt2 = gp_Pnt2d(0., 120.)
#    height = 100.
#    pl = Geom_Plane(gp_Pln())
#    for i in range(0, 40):
#        # TODO: the parameter slope needs to be visualized
#        slope = i/100.
#        bc = batten_curve(pt1, pt2, height, slope,
#                          math.radians(i), math.radians(-i))
#        display.EraseAll()
#        edge = BRepBuilderAPI_MakeEdge(bc, pl.GetHandle()).Edge()
#        display.DisplayShape(edge, update=True)
#        time.sleep(0.21)


# These functions are from the core_geometry_util examples in pythonocc-core
def make_edge(*args):
    edge = BRepBuilderAPI_MakeEdge(*args)
    with assert_isdone(edge, 'failed to produce edge'):
        result = edge.Edge()
        edge.Delete()
        return result


def make_wire(*args):
    # if we get an iterable, than add all edges to wire builder
    if isinstance(args[0], list) or isinstance(args[0], tuple):
        wire = BRepBuilderAPI_MakeWire()
        for i in args[0]:
                wire.Add(i)
        wire.Build()
        return wire.Wire()

    wire = BRepBuilderAPI_MakeWire(*args)
    wire.Build()
    with assert_isdone(wire, 'failed to produce wire'):
        result = wire.Wire()
        return result


def make_face(*args):
    face = BRepBuilderAPI_MakeFace(*args)
    with assert_isdone(face, 'failed to produce face'):
        result = face.Face()
        face.Delete()
        return result


def make_pipe_shell(spine, profiles, support=None):
    try:
        spine = make_wire(make_edge(spine))
    except:
        pass
    pipe = BRepOffsetAPI_MakePipeShell(spine)
    for profile in profiles:
        try:
            pipe.Add(profile)
        except:
            wire = make_wire(make_edge(profile))
            pipe.Add(wire)
    if support:
        pipe.SetMode(support)
    pipe.Build()
    with assert_isdone(pipe, 'failed building pipe'):
        return pipe.Shape()


def make_vertex(*args):
    vert = BRepBuilderAPI_MakeVertex(*args)
    result = vert.Vertex()
    return result


def make_ellipsoid(centre_pt, dx, dy, dz):
    """Creates an ellipsoid from non-uniformly scaled unit sphere"""
    sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), 0.5)
    ellipsoid = transform_nonuniformal(sphere.Shape(), [dx, dy, dz],
                                       vec=centre_pt)
    return ellipsoid


def make_circle3pt(pt1, pt2, pt3):
    """Makes a circle allowing python lists as input points"""
    try:
        pt1 = gp_Pnt(*pt1)
        pt2 = gp_Pnt(*pt2)
        pt3 = gp_Pnt(*pt3)
    except:
        pass
    return GC_MakeCircle(pt1, pt2, pt3).Value()


def PlanarSurf(geomcurve):
    """Adds a planar surface to curve

    Parameters
    ----------
    geomcurve : OCC.Geom type curve
        The edge of the profile

    Returns
    -------
    surf : TopoDS_face
        the planar surface
    """
    try:
        wire = make_wire(make_edge(geomcurve))
    except:
        # If the geomcurve is passed directly rather than by it's handle:
        wire = make_wire(make_edge(geomcurve.GetHandle()))
    face = make_face(wire)
    return face


def project_curve_to_surface(curve, surface, dir):
    '''
    Returns a curve as projected onto the surface shape

    Parameters
    ----------
    curve : Geom_curve or TopoDS_Edge/Wire
    
    surface : TopoDS_Shape

    dir : gp_Dir
        the direction of projection

    Returns
    -------
    res_curve : geom_curve (bspline only?)
    '''
    try:
        edge = make_edge(curve)
    except:
#        if converting to edge didn't work, assume curve is already an edge
        edge = curve
    
    wire = make_wire(edge)   # This will return wire is curve is already a wire
    from OCC.BRepProj import BRepProj_Projection
    from OCC.BRepAdaptor import BRepAdaptor_CompCurve
    proj = BRepProj_Projection(wire, surface, dir)
    res_wire = proj.Current()
    res_curve = BRepAdaptor_CompCurve(res_wire).BSpline()
    return res_curve


def points_from_intersection(plane, curve):
    '''
    Find intersection points between plane and curve.

    Parameters
    ----------
    plane : Geom_Plane
        The Plane

    curve : Geom_*Curve
        The Curve

    Returns
    -------
    P : Point or list of points
        A single intersection point (OCC.gp.gp_Pnt) if one intersection is
        found, or list of points if more than one is found.
            - If No Intersection points were found, returns None

    Notes
    -----
    The plane is first converted to a surface As the GeomAPI_IntCS class
    requires this.
    '''
    intersector = GeomAPI_IntCS(curve.GetHandle(), plane.GetHandle())

    with assert_isdone(intersector, 'failed to calculate intersection'):
        nb_results = intersector.NbPoints()
        if nb_results == 1:
            return intersector.Point(1)
        elif nb_results >= 1:
            P = []
            for i in range(1, nb_results + 1):
                P.append(intersector.Point(i))
        else:
            return None


# TODO: Network surface function needs fixing
# def Add_Network_Surface(curvenet, deg=3, initsurf=None):
#     '''Adds a surface from curve network using the OCC plate surface algorithm
    
#     This function is in development and currently raises an error 

#     Parameters
#     ----------
#     curvenet : list of Handle_GeomCurve

#     Notes
#     -----
#     '''
#     raise NotImplementedError('This function is not yet safe for general use')
# #    fill = BRepFill_Filling(deg)
# #    for curve in curvenet:
# #        try:
# #            fill.Add(make_edge(curve), continuity)
# #        except TypeError:
# #            # If curve is given as object rather than handle
# #            fill.Add(make_edge(curve.GetHandle()), continuity)
# #    fill.Build()
# #    face = fill.Face()
# #    return face
#     print("This function is not tested and should not be used with certainty")
#     builder = GeomPlate_BuildPlateSurface(deg, 15, 5)
#     if initsurf is not None:
#         "Loading Initial Surface"
#         builder.LoadInitSurface()
#         "Initial Surface loaded"

#     for curve in curvenet:
#         print(type(curve))
#         adaptor = GeomAdaptor_Curve(curve)
#         Hadapter = GeomAdaptor_HCurve(adaptor)
#         constr = GeomPlate_CurveConstraint(Hadapter.GetHandle(), 0)
#         builder.Add(constr.GetHandle())
#         # first didnt work... attempt 2 :
# #        edge = make_edge(curve)
# #        C = BRepAdaptor_HCurve()
# #        C.ChangeCurve().Initialize(edge)
# #        Cont = BRepFill_CurveConstraint(C.GetHandle(), 0).GetHandle()
# #        builder.Add(Cont)
# #
# #    Try adding from wires instead.. attempt 3:
# #         exp =

#     builder.Perform()
#     with assert_isdone(builder, 'Failed to create Plate Surface'):
#         # Approximate the surface into a bspline surface
#         surf = builder.Surface()
#         approx = GeomPlate_MakeApprox(surf, 0.001, 10, 8, 0.001, 0).Surface()
#         Umin, Umax, Vmin, Vmax = surf.GetObject().Bounds()
#         print(Umin, Umax, Vmin, Vmax)
#         print("about to make face:")
#         face = make_face(approx, 0.1)   # Umin, Umax, Vmin, Vmax, 0.1)
#         print("Face made")
#         return face


def CutSect(Shape, SpanStation):
    """
    Parameters
    ----------
    Shape : TopoDS_Shape
        The Shape to find planar cut section (parallel to xz plane)

    SpanStation : scalar in range (0, 1)
        y-direction location at which to cut Shape

    Returns
    -------
    Section : result of OCC.BRepAlgoAPI.BRepAlgoAPI_Section (TopoDS_Shape)
        The cut section of shape given a cut plane parallel to xz at input
        Spanstation.

    Chord : result of OCC.GC.GC_MakeSegment.Value (Geom_TrimmedCurve)
        The Chord line between x direction extremeties
    """
    (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax) = ObjectsExtents([Shape])

    YStation = Ymin + (Ymax - Ymin) * SpanStation
    OriginX = Xmin - 1
    OriginZ = Zmin - 1

    P = gp_Pln(gp_Pnt(OriginX, YStation, OriginZ), gp_Dir(gp_Vec(0, 1, 0)))
    # Note: using 2*extents here as previous +1 trimmed plane too short
    CutPlaneSrf = make_face(P, 0, Zmax + 2, 0, Xmax +2)

    I = BRepAlgoAPI_Section(Shape, CutPlaneSrf)
    I.ComputePCurveOn1(True)
    I.Approximation(True)
    I.Build()
    Section = I.Shape()

    (Xmin, Ymin, Zmin, Xmax, Ymax, Zmax) = ObjectsExtents([Section])

#     Currently assume only one edge exists in the intersection:
    exp = TopExp_Explorer(Section, TopAbs_EDGE)
    edge = topods_Edge(exp.Current())

#    Find the apparent chord of the section (that is, the line connecting the
#    fore most and aftmost points on the curve
    DivPoints = Uniform_Points_on_Curve(edge, 200)

    Xs = np.array([pt.X() for pt in DivPoints])

    min_idx = np.argmin(Xs)
    LeadingPoint = gp_Pnt(Xs[min_idx], DivPoints[min_idx].Y(),
                          DivPoints[min_idx].Z())

    max_idx = np.argmax(Xs)
    TrailingPoint = gp_Pnt(Xs[max_idx], DivPoints[max_idx].Y(),
                           DivPoints[max_idx].Z())

    HChord = GC_MakeSegment(TrailingPoint, LeadingPoint).Value()
#    Chord = HChord.GetObject()
    return Section, HChord


def AddCone(BasePoint, Radius, height, direction=gp_Dir(1, 0, 0)):
    """Generates a cone shape originating at BasePoint with base Radius
    and height (points in the direction of input 'direction)

    Parameters
    ----------
    BasePoint : OCC.gp.gp_Pnt or array length 3
        The centre base point
    
    Radius : scalar
        Cone base radius
    
    height : scalar
        Cone height

    direction : OCC.gp.gp_Dir  (default: positive x direction)
        the direction of the cones axis i.e. normal to the base:
        defaults to x axis
    
    Returns
    -------
    shape : TopoDS_Shape
        The generated Cone
    """
    try:
        BasePoint = gp_Pnt(*BasePoint)
    except:
        pass
    ax2 = gp_Ax2(BasePoint, direction)
    cone = BRepPrimAPI_MakeCone(ax2, Radius, 0, height)
    return cone.Shape()


def TrimShapebyPlane(Shape, Plane, pnt=gp_Pnt(0, -10, 0)):
    """Trims an OCC shape by plane. Default trims the negative y side of the
    plane

    Parameters
    ----------
    Shape : TopoDS_Shape
    
    Plane : expect TopoDS_Face
    
    pnt : point defining which side of the halfspace contains its mass
    """
    tool = BRepPrimAPI_MakeHalfSpace(Plane, pnt).Solid()
    trimmed_shape = boolean_cut(Shape, tool)

    return trimmed_shape


def boolean_cut(shapeToCutFrom, cuttingShape):
    """Boolean cut tool from PythonOCC-Utils"""
    try:
        cut = BRepAlgoAPI_Cut(shapeToCutFrom, cuttingShape)
        print 'can work?', cut.BuilderCanWork()
        _error = {0: '- Ok',
                  1: '- The Object is created but Nothing is Done',
                  2: '- Null source shapes is not allowed',
                  3: '- Check types of the arguments',
                  4: '- Can not allocate memory for the DSFiller',
                  5: '- The Builder can not work with such types of arguments',
                  6: '- Unknown operation is not allowed',
                  7: '- Can not allocate memory for the Builder',
                  }
        print 'error status:', _error[cut.ErrorStatus()]
#        cut.RefineEdges()
        shp = cut.Shape()
        cut.Destroy()
        return shp
    except:
        print 'FAILED TO BOOLEAN CUT'
        return shapeToCutFrom
