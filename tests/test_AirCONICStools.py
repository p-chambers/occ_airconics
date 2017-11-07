# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 13:49:32 2016

@author: pchambers
"""
import airconics.AirCONICStools as act
import numpy as np
from OCC.gp import gp_Pnt, gp_Dir, gp_Vec
import pytest
from airconics import LiftingSurface, Fuselage

def test_coslin():
    abscissa = act.coslin(0.5, 8, 8)[0]
    ans = np.array([0.0,
                    0.01253604390908819,
                    0.04951556604879043,
                    0.1090842587659851,
                    0.1882550990706332,
                    0.2830581304412209,
                    0.3887395330218428,
                    0.49999999999999994,
                    0.5625,
                    0.625,
                    0.6875,
                    0.75,
                    0.8125,
                    0.875,
                    0.9375,
                    1.0])
    assert(np.all(np.abs(abscissa - ans) < 1e-10))


def test_Objects_Extents():
    box = act.BRepPrimAPI_MakeBox(1, 1, 1).Shape()
    X = np.array(act.ObjectsExtents(box))
    expected = np.array([0, 0, 0, 1, 1, 1])
    assert(np.all(np.abs(X - expected) < 1e-5))


def test_points_to_bspline_nparray():
    # This is just to check my function will succeed to create a bspline - 
    #  Hopefully PythonOCC tests will test the values of curves generated
    pnts = np.array([[0, 0, 0], [1, 0, 2], [2, 0, 3], [4, 0, 3], [5, 0, 5]])
    spline = act.points_to_bspline(pnts)

    # test the curve is not null
    assert(not spline.IsNull())

    # This will raise an error if the curve creation didn't work - don't need
    #  to add an assert statement here
    o = spline.GetObject()


def test_project_curve_to_plane():
    # Projects a line of length 1 from above the XOY plane, and tests points
    # on the resulting line
    from OCC.Geom import Geom_Plane, Geom_TrimmedCurve
    from OCC.GC import GC_MakeSegment
    from OCC.gp import gp_Ax3, gp_XOY, gp_Pnt, gp_Dir
    XOY = Geom_Plane(gp_Ax3(gp_XOY()))
    curve = GC_MakeSegment(gp_Pnt(0, 0, 5),
                           gp_Pnt(1, 0, 5)).Value()
    direction = gp_Dir(0, 0, 1)

    Hproj_curve = act.project_curve_to_plane(curve, XOY.GetHandle(),
                                            direction)

    proj_curve = Hproj_curve.GetObject()

    # The start and end points of the curve
    p1 = proj_curve.Value(0)
    p2 = proj_curve.Value(1)

    p1_array = np.array([p1.X(), p1.Y(), p1.Z()])
    p2_array = np.array([p2.X(), p2.Y(), p2.Z()])

    # The expected start and end points
    start = np.array([0, 0, 0])
    end = np.array([1, 0, 0])

    # Assert that neither points have a Y or Z component, and that
    assert((np.all(p1_array == start) and np.all(p2_array == end)) or
           (np.all(p1_array == end) and np.all(p2_array == start)))


def test_project_curve_to_surface():
    """Returning object rather than handle from this function gave bugs
    between 0.16.3 and 0.16.5 - adding test to preempt future oddness"""
    # Edge to project:
    h = 5
    pnts = np.array([[0, h, 0], [1, h, 2], [2, h, 3], [4, h, 3], [5, h, 5]])
    spline = act.points_to_bspline(pnts)

    # Surface to project onto:
    from OCC.GC import GC_MakeCircle
    circle = GC_MakeCircle(gp_Pnt(0,0,0), gp_Dir(0, 1, 0), 10).Value()
    edge = act.make_edge(circle)
    face = act.make_face(act.make_wire(edge))

    Hprojected_curve = act.project_curve_to_surface(spline, face,
                                                  gp_Dir(0, -1, 0))

    # Check that the curve created is not null
    check_valid_object(Hprojected_curve)


def test_CalculateSurfaceArea():
    """Use a few simple shapes to test the surface area function.

    Notes
    -----
    This isnt testing the area building algorithm, just that my setup of the
    function works for a variety of different OCC objects - PChambers
    """
    # A flat square edge lengths 1
    from OCC.BRepBuilderAPI import BRepBuilderAPI_MakePolygon
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(1, 0, 0)
    p3 = gp_Pnt(1, 1, 0)
    p4 = gp_Pnt(0, 1, 0)
    surf1 = act.make_face(
        BRepBuilderAPI_MakePolygon(p1, p2, p3, p4, True).Wire())
    # The tolerance for difference between output and expected area
    tol = 1e-12
    assert(np.abs(act.CalculateSurfaceArea(surf1) - 1) < tol)

    # A sphere with radius 1
    from OCC.BRepPrimAPI import BRepPrimAPI_MakeSphere
    r = 1
    # The tolerance need to be relaxed a bit for this case
    tol = 1e-04
    sphere = BRepPrimAPI_MakeSphere(r).Shape()
    assert(np.abs(act.CalculateSurfaceArea(sphere) - 4 * np.pi * (r**2)) < tol)


def test_CalculateVolume():
    """Use a few simple shapes to test the surface area function.

    Notes
    -----
    This isnt testing the area building algorithm, just that my setup of the
    function works for a variety of different OCC objects - PChambers
    """
    # A flat square edge lengths 1
    from OCC.BRepBuilderAPI import BRepBuilderAPI_MakePolygon
    p1 = gp_Pnt(0, 0, 0)
    p2 = gp_Pnt(1, 0, 0)
    p3 = gp_Pnt(1, 1, 0)
    p4 = gp_Pnt(0, 1, 0)
    surf1 = act.make_face(
        BRepBuilderAPI_MakePolygon(p1, p2, p3, p4, True).Wire())


    p5 = gp_Pnt(0, 0, 5)
    p6 = gp_Pnt(1, 0, 5)
    p7 = gp_Pnt(1, 1, 5)
    p8 = gp_Pnt(0, 1, 5)
    surf1 = act.make_wire(
        BRepBuilderAPI_MakePolygon(p5, p6, p7, p8, True).Wire())

    # The tolerance for difference between output and expected area
    tol = 1e-8
    assert(np.abs(act.CalculateVolume(surf1) - 0.0) < tol)

    # A sphere with radius 1
    from OCC.BRepPrimAPI import BRepPrimAPI_MakeSphere
    r = 1
    # The tolerance need to be relaxed a bit for this case
    sphere = BRepPrimAPI_MakeSphere(r).Shape()
    assert(np.abs(act.CalculateVolume(sphere) - 4 * np.pi * r**3 / 3) < tol)


def test_CalculateVolume_LiftingSurface():
    """Tests that the closed solid volume of a simple straight lifting surface
    is approximately equal to the area of the root airfoil multiplied by its
    span"""
    from airconics.examples.straight_wing import *
    apex = [0., 0., 0.]
    Wing = LiftingSurface(apex, SimpleSweepFunction,
                          SimpleDihedralFunction,
                          SimpleTwistFunction,
                          SimpleChordFunction,
                          SimpleAirfoilFunction,
                          SegmentNo=1,
                          ScaleFactor=5.,
                          ChordFactor=2.)
    test_vol = act.CalculateSurfaceArea(Wing['Root']) * Wing.ActualSemiSpan
    assert(abs(
        (act.CalculateVolume(Wing.MakeSolid()) - test_vol) / test_vol) < 1e-2)


def test_CalculateVolume_Fuselage():
    """Tests that the closed solid volume of a simple straight lifting surface
    is approximately equal to the area of the root airfoil multiplied by its
    span"""
    from airconics.examples.straight_wing import *
    Fus = Fuselage(NoseLengthRatio=0.182,
                   TailLengthRatio=0.293,
                   Scaling=[1.0, 1.0, 1.0],
                   NoseCoordinates=[0., 0., 0],
                   CylindricalMidSection=False,
                   Maxi_attempt=5)

    # Perform a simple numerical optimization on the fuselage volume, using an
    # approximation to the average area.
    test_vol = 0
    Scaling = np.array(Fus.Scaling) / 55.902
    for i in range(len(Fus.SectionAreas[:-1])):
        dx = (Fus.StationRange[i+1] - Fus.StationRange[i])
        test_vol += ((Fus.SectionAreas[i] ** 0.5 +
            Fus.SectionAreas[i+1] ** 0.5) / 2.) ** 2 * dx
    test_vol *= np.prod(Scaling)
    # Hope that at maximum 1% out...
    assert(abs(
        (act.CalculateVolume(Fus.MakeSolid()) - test_vol) / test_vol) < 1)


# Misc functions
def check_valid_object(handle):
    """Tests that the object pointed to by input handle doesn't break stuff"""
    # test the handle is not null
    assert(not handle.IsNull())

    # This will raise an error if the object creation didn't work - don't need
    #  to add an assert statement here
    o = handle.GetObject()
