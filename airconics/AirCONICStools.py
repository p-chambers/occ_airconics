# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 11:58:52 2015

@author: pchambers
"""
# Geometry Manipulation libraries:
import OCC.Bnd
from OCC.ShapeConstruct import shapeconstruct
from OCC.GeomAPI import GeomAPI_PointsToBSpline
from OCC.BRepBndLib import brepbndlib_Add
from OCC.TColgp import TColgp_Array1OfPnt
from OCC.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeEdge,
                                BRepBuilderAPI_Transform)
from OCC.gp import gp_Trsf, gp_Lin

# FileIO libraries:
from OCC.STEPControl import STEPControl_Writer, STEPControl_AsIs, \
                             STEPControl_Reader
from OCC.Interface import Interface_Static_SetCVal
from OCC.IFSelect import IFSelect_RetDone  #, IFSelect_ItemsByEntity

# Standard Python libraries
import numpy as np

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

    BB = OCC.Bnd.Bnd_Box()
    BB.SetGap(tol)
    
    for shape in ObjectIds:
        brepbndlib_Add(shape, bbox)
        
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    if as_vec is False:
        return xmin, ymin, zmin, xmax, ymax, zmax
    else:
        return gp_Vec(xmin, ymin, zmin), gp_Vec(xmax, ymax, zmax)


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


def points_to_bspline(pnts):
    """
    Points to bspline: obtained from pythonocc-utils
    """
    pnts = point_list_to_TColgp_Array1OfPnt(pnts)
    crv = GeomAPI_PointsToBSpline(pnts)
    return crv.Curve()


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


def AddSurfaceLoft(objs):
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

    generator.Build()
    return generator.Shape()


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