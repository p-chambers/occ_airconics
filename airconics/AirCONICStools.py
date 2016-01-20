# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 11:58:52 2015

@author: pchambers
"""
# Geometry Manipulation libraries:
import OCC.Bnd
from OCC.AIS import AIS_WireFrame, AIS_Shape
from OCC.ShapeConstruct import shapeconstruct
from OCC.Geom import Geom_BezierCurve
from OCC.GeomAPI import GeomAPI_PointsToBSpline
from OCC.BRepBndLib import brepbndlib_Add
from OCC.TColgp import TColgp_Array1OfPnt
from OCC.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeEdge,
                                BRepBuilderAPI_Transform,
                                BRepBuilderAPI_MakeFace)
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.TopoDS import TopoDS_Shape
from OCC.gp import gp_Trsf, gp_Lin, gp_Ax2, gp_Pnt, gp_Dir, gp_Vec
from OCC.GeomAbs import GeomAbs_C2

# FileIO libraries:
from OCC.STEPControl import STEPControl_Writer, STEPControl_AsIs, \
                             STEPControl_Reader
from OCC.Interface import Interface_Static_SetCVal
from OCC.IFSelect import IFSelect_RetDone  #, IFSelect_ItemsByEntity


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


def ObjectsExtents(ObjectIds, tol=1e-6, as_vec=False):
    """Compute the extents in the X, Y and Z direction (in the current
    coordinate system) of the objects listed in the argument.

    Parameters
    ----------
    ObjectIds -

    Returns
    -------
    if `as_vec` is True, return a tuple of gp_Vec instances
         for the lower and another for the upper X,Y,Z values representing the
         bounding box

    if `as_vec` is False, return a tuple of lower and then upper X,Y,Z values
         representing the bounding box
    """

    bbox = OCC.Bnd.Bnd_Box()
    bbox.SetGap(tol)

    for shape in ObjectIds:
        brepbndlib_Add(shape, bbox)

    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    if as_vec is False:
        return xmin, ymin, zmin, xmax, ymax, zmax
    else:
        return gp_Vec(xmin, ymin, zmin), gp_Vec(xmax, ymax, zmax)


def BBox_FromExtents(xmin, ymin, zmin, xmax, ymax, zmax):
    """Generates the Wire Edges defining the Bounding Box defined in the input
    arguments: Can be used to display the bounding box"""
    s = BRepPrimAPI_MakeBox(gp_Pnt(xmin, ymin, zmin), gp_Pnt(xmax, ymax, zmax)).Shape()
    ais_bbox = AIS_Shape(s)
    ais_bbox.SetDisplayMode(AIS_WireFrame)
    return ais_bbox.GetHandle()


#def _Tcol_dim_1(li, _type):
#    """
#    Function factory for 1-dimensional TCol* types: pythonocc-utils
#    """
#    pts = _type(0, len(li)-1)
#    for n, i in enumerate(li):
#        pts.SetValue(n, i)
#    return pts


def point_list_to_TColgp_Array1OfPnt(li):
    pts = TColgp_Array1OfPnt(0, len(li)-1)
    for n, i in enumerate(li):
        pts.SetValue(n, i)
    return pts

def array_to_TColgp_Array1OfPnt(array):
    """Function to return curve from numpy array
    Parameters
    ----------
    array - array (Npts x 3)
        Array of xyz points for which to fit a bspline
    Returns
    -------
    pt_arr - TCOLgp_Array1OfPnt
        OCC type array of points
    """
    dims = array.shape
    # Attempt to tranpose if x, y, z points span columns (should span rows)
    if dims[0] == 3:
        array = array.T
    elif dims[1] != 3:
        raise ValueError("Array must have dimension Npnts x 3 (x, y, z)")
    N = array.shape[0]
    pt_arr = TColgp_Array1OfPnt(0, N-1)
    for i, pt in enumerate(array):
        pt_arr.SetValue(i, gp_Pnt(*pt))
    return pt_arr


def points_to_bspline(pnts):
    """
    Points to bspline: originally from pythonocc-utils, changed to allow numpy
    arrays as input
    """
    if type(pnts) is np.ndarray:
        pnts = array_to_TColgp_Array1OfPnt(pnts)
    elif type(pnts) is list:
        if type(pnts[0]) is gp_Pnt:
            pass
        else:
            assert(len(pnts[0]) == 3),  ("""Points should have length 3 (x, y, z).
                  Found {}""".format(len(pnts[0])))
            pnts = [gp_Pnt(pt) for pt in pnts]
        pnts = point_list_to_TColgp_Array1OfPnt(pnts)

    # Fit the curve to the point array
    crv = GeomAPI_PointsToBSpline(pnts)
    return crv.Curve()


def points_to_BezierCurve(pnts):
    """
    Creates a Bezier curve from an array of points.
    
    Parameters
    ----------
    pnts - array or list
        x, y, z for an array of points. Allowable inputs are numpy arrays
        (with dimensions (Npoints x 3)), python list with elements [xi, yi, zi]
        or list of OCC.gp.gp_Pnt objects
    """
    if type(pnts) is np.ndarray:
        pnts = array_to_TColgp_Array1OfPnt(pnts)
    elif type(pnts) is list:
        if type(pnts[0]) is gp_Pnt:
            pass
        else:
            assert(len(pnts[0]) == 3),  ("""Points should have length 3 (x, y, z).
                  Found {}""".format(len(pnts[0])))
            pnts = [gp_Pnt(pt) for pt in pnts]
        pnts = point_list_to_TColgp_Array1OfPnt(pnts)

    # Fit the curve to the point array
    crv = Geom_BezierCurve(pnts)
    return crv


def scale_uniformal(brep, pnt, factor, copy=False):
    '''
    translate a brep over a vector : from pythonocc-utils
    Paramters
    ---------
    brep - TopoDS_Shape
        the TopoDS_Shape to scale
    pnt - gp_Pnt
        Origin of scaling
    factor - scalar
        Scaling factor
    copy - bool
        copies to brep if True
    '''
    trns = gp_Trsf()
    trns.SetScale(pnt, factor)
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def coslin(TransitionPoint, NCosPoints=8, NLinPoints=8):
    """Creates a series of abscissas with cosine spacing from 0 to a
    TransitionPoint and a linear spacing thereafter, up to 1. The
    TransitionPoint corresponds to pi. Distribution suitable for airfoils
    defined by points. TransitionPoint must be in the range [0,1].
    Parameters
    ----------

    Returns
    -------
    """
    angles = np.linspace(0, np.pi/2., NCosPoints)
    cos_pts = TransitionPoint * (1.-np.cos(angles))
    lin_pts = np.linspace(TransitionPoint, 1., NLinPoints+1)
    # Combine points and Remove first linear point (already in cos_pts):
    Abscissa = np.hstack((cos_pts, lin_pts[1:]))
    return Abscissa, NCosPoints


def export_STEPFile(shapes, filename):
    # initialize the STEP exporter
    step_writer = STEPControl_Writer()
    Interface_Static_SetCVal("write.step.schema", "AP203")

    # transfer shapes
    for shape in shapes:
        step_writer.Transfer(shape, STEPControl_AsIs)
    
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


def AddSurfaceLoft(objs, continuity=GeomAbs_C2, check_compatibility=True):
    """Create a lift surface through curve objects
    Parameters
    ----------
    objs - list of python classes
        Each obj is expected to have an obj.Curve attribute :
        see airconics.primitives.airfoil class
    Returns
    -------
    """
    assert(len(objs) >= 2), 'Loft Failed: Less than two input curves'
    # Note: This is to give a smooth loft.
    isSolid = False; ruled = False; pres3d=1e-08
    args = [isSolid, ruled, pres3d]    # args (in order) for ThruSections
    generator = BRepOffsetAPI_ThruSections(*args)
    
    for obj in objs:
        try:
            edge = make_edge(obj.Curve)
            generator.AddWire(BRepBuilderAPI_MakeWire(edge).Wire())
        except AttributeError:
            print("""Warning: one or more object has no 'Curve' attribute and
            could not be added to the loft""")
            continue
    
    generator.CheckCompatibility(check_compatibility)
    generator.SetContinuity(continuity)
    generator.Build()
    with assert_isdone(generator, 'failed lofting'):
        return generator.Shape()
#    return generator.Shape()


def Generate_InterpFunction(Values, EpsArray=None, uniform=True):
    """Given an array of spanwise coordinates epsilon along a curvilinear
    leading-edge attached coordinate system, and a set of values describing
    e.g. Chord, Sweep at each station, generate and return a function
    f(epsilon) which will give the interpolated value.
    Parameters
    ----------
    EpsArray - array of float
        Distribution of spanwise coordinates at which the Values are known
    Values - array of float
        Values of e.g. chordlength, sweep at each spanwise location in EpsArray
    uniform - bool
        If True, assumes that Values corresponds to uniformly distribution
        epsilon locations along the lifting surface span
    """
    if uniform:
        EpsArray = np.linspace(0, 1, len(Values))

    def f(Epsilon):
        return np.interp(Epsilon, EpsArray, Values)

    return f


def translate_topods_from_vector(brep_or_iterable, vec, copy=False):
    '''
    Function Originally from pythonocc-utils, modified to work on objects

    translate a brep over a vector
    Paramters
    ---------
    brep - the Topo_DS to translate
    vec-  the vector defining the translation
    copy - copies to brep if True
    '''
    trns = gp_Trsf()
    trns.SetTranslation(vec)
    if issubclass(brep_or_iterable.__class__, TopoDS_Shape):
        brep_trns = BRepBuilderAPI_Transform(brep_or_iterable, trns, copy)
        brep_trns.Build()
        return brep_trns.Shape()
    else:
        return [translate_topods_from_vector(brep_or_iterable, vec, copy) for i in brep_or_iterable]


def rotate(brep, axe, degree, copy=False):
    '''
    Originally from pythonocc-utils : might add dependency on this?

    brep - shape to rotate
    axe - axis of rotation
    degree - Number of degrees to rotate through
    '''
    trns = gp_Trsf()
    trns.SetRotation(axe, np.radians(degree))
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    brep_trns.Build()
    return brep_trns.Shape()


def mirror(brep, plane='xz', axe2=None, copy=False):
    """Originally from pythonocc-utils : might add dependency on this?
    Mirror object
    Params
    ------
    
    Returns
    -------
    
    Notes
    -----
    I added a functionality here to specify a plane using a string so that
    users could avoid interacting with core occ objects"""
    if axe2:
        plane = None
    else:
        Orig = gp_Pnt(0., 0., 0.)
        xdir = gp_Dir(1, 0, 0)
        ydir = gp_Dir(0, 1, 0)
        zdir = gp_Dir(0, 0, 1)
        
        if plane == 'xz':
            axe2 = gp_Ax2(Orig, ydir)
        elif plane == 'yz':
            axe2 = gp_Ax2(Orig, xdir)
        elif plane == 'xy':
            axe2 = gp_Ax2(Orig, zdir)
        else:
            raise(AssertionError, "Unknown mirror plane string,", plane)
    trns = gp_Trsf()
    trns.SetMirror(axe2)
    brep_trns = BRepBuilderAPI_Transform(brep, trns, copy)
    return brep_trns.Shape()


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
    result = edge.Edge()
    return result


def make_wire(*args):
    # if we get an iterable, than add all edges to wire builder
    if isinstance(args[0], list) or isinstance(args[0], tuple):
        wire = BRepBuilderAPI_MakeWire()
        for i in args[0]:
            wire.Add(i)
        wire.Build()
        print(wire)
        return wire.Wire()
    wire = BRepBuilderAPI_MakeWire(*args)
    return wire.Wire()


def make_face(*args):
    face = BRepBuilderAPI_MakeFace(*args)
    with assert_isdone(face, 'failed to produce face'):
        result = face.Face()
        face.Delete()
        return result