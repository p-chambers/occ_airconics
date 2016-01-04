# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 11:58:52 2015

@author: pchambers
"""
# Geometry Manipulation libraries:
from OCC.BRepBndLib import brepbndlib_Add
import OCC.Bnd
from OCC.TColgp import TColgp_Array1OfPnt
from OCC.BRepOffsetAPI import BRepOffsetAPI_ThruSections
from OCC.BRepBuilderAPI import (BRepBuilderAPI_MakeWire,
                                BRepBuilderAPI_MakeEdge)

# FileIO libraries:
from OCC.STEPControl import STEPControl_Writer, STEPControl_AsIs
from OCC.Interface import Interface_Static_SetCVal
from OCC.IFSelect import IFSelect_RetDone

# Standard Python libraries
import numpy as np


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


def _Tcol_dim_1(li, _type):
    """
    Function factory for 1-dimensional TCol* types
    """
    pts = _type(0, len(li)-1)
    for n, i in enumerate(li):
        pts.SetValue(n, i)
    return pts


def point_list_to_TColgp_Array1OfPnt(li):
    """
    List of gp_Pnt2d to TColgp_Array1OfPnt2d
    """
    return _Tcol_dim_1(li, TColgp_Array1OfPnt)


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


def surface_from_airfoils(Airfoil1, Airfoil2):
    """Create a lift surface through airfoils (currently expecting 2 only)
    Parameters
    ----------
    
    Returns
    -------
    """
    generator = BRepOffsetAPI_ThruSections(False, True)
    for i, Airfoil in enumerate([Airfoil1, Airfoil2]):
        edge = make_edge(Airfoil.Curve)
        generator.AddWire(BRepBuilderAPI_MakeWire(edge).Wire())
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