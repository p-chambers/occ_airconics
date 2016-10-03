# -*- coding: utf-8 -*-
"""
Created on Thu Jan 14 15:10:50 2016

@author: pchambers
"""
import numpy as np
from airconics.liftingsurface import LiftingSurface, airfoilfunct
from airconics.primitives import Airfoil
import airconics.AirCONICStools as act
from airconics.examples.wing_example_transonic_airliner import *
from airconics.examples.straight_wing import *

from OCC.gp import gp_Pnt
import pytest


@pytest.fixture
def simple_wing():
    """Initialises a simple wing class, with zero twist, zero sweep, zero taper
    zero dihedral, AR=5, chord=1"""
    NSeg = 11
    wing = LiftingSurface(ChordFunct=SimpleChordFunction,
                          DihedralFunct=SimpleDihedralFunction,
                          SweepFunct=SimpleSweepFunction,
                          AirfoilFunct=SimpleAirfoilFunction,
                          TwistFunct=SimpleTwistFunction,
                          ScaleFactor=5,
                          ChordFactor=0.2,
                          SegmentNo=NSeg)
    return wing


def test_empty_liftingsurface():
    """Tests that construction"""
    wing = LiftingSurface(construct_geometry=False)


def test_empty_liftingsurface_Build_error():
    """Tests whether building a lifting surface with undefined functional
    parameters raises an error"""
    with pytest.raises(AssertionError):
        wing = LiftingSurface(construct_geometry=False)
        # No f(eps) parameters have been defined so this should raise an error
        wing.Build()


def test_default_args():
    """Tests that the default arguments do not raise an error. Note that
    no default surface is expected to be generated however"""
    wing = LiftingSurface()
    assert(len(wing) == 0)
    assert(len(wing.Sections) == 0)


def test_GenerateLeadingEdge():
    # Test if GenerateLeadingEdge function matches previous version for
    # the Transonic Airliner example: Note this is only to 3dp
    LEPoints_ref = np.array([[0.000e+00,   0.000e+00,   0.000e+00],
                            [9.088e-02,   2.147e-03,   2.644e-04],
                            [1.752e-01,   3.580e-02,   4.508e-03],
                            [2.274e-01,   1.096e-01,   1.425e-02],
                            [2.795e-01,   1.834e-01,   2.463e-02],
                            [3.317e-01,   2.570e-01,   3.586e-02],
                            [3.838e-01,   3.304e-01,   4.815e-02],
                            [4.359e-01,   4.037e-01,   6.171e-02],
                            [4.881e-01,   4.766e-01,   7.675e-02],
                            [5.402e-01,   5.492e-01,   9.346e-02],
                            [5.924e-01,   6.213e-01,   1.121e-01],
                            [6.707e-01,   6.655e-01,   1.248e-01]])

    # Predefined Transonic Airliner example functions used here
    P = [0., 0., 0.]
    NSeg = 11
    Wing = LiftingSurface(P, mySweepAngleFunctionAirliner,
                             myDihedralFunctionAirliner,
                             myTwistFunctionAirliner,
                             myChordFunctionAirliner,
                             myAirfoilFunctionAirliner, SegmentNo=NSeg)

    LEPoints = Wing.GenerateLeadingEdge()
    # Test First and final point first (easier to debug)
    assert(np.all(np.abs(LEPoints[1] - LEPoints_ref[1]) < 1e-3))
    assert(np.all(np.abs(LEPoints[-1] - LEPoints_ref[-1]) < 1e-3))

    # Now check that the entire array is less than 3 dp. out:
    assert(np.all(np.abs(LEPoints - LEPoints_ref) < 1e-3))


@pytest.mark.xfail
def test_liftingSurface_ChordScaleOptimizer():
    raise NotImplementedError


def test_GenerateSectionCurves_Numpy_Functs():
    """Tests the GenerateSectionCurves function given a set of Lifting surface
    functional parameters f(eps) which expects a numpy array """
    def myNumpyFunct(eps):
        # This will be used for twist, sweep. dihedral, and chord. returns
        # an array, as eps is expected to be an array
        return np.ones_like(eps) * 5
    wing = LiftingSurface(construct_geometry=False)
    wing.ChordFunct = myNumpyFunct
    wing.SweepFunct = myNumpyFunct
    wing.DihedralFunct = myNumpyFunct
    wing.TwistFunct = myNumpyFunct
    wing.AirfoilFunct = myAirfoilFunctionAirliner   # Builtin example
    wing.GenerateSectionCurves()
    assert(len(wing._Sections) > 0)


def test_GenerateSectionCurves_NonNumpy_ChordFunct():
    """Tests the GenerateSectionCurves function given a set of Lifting surface
    functional parameters f(eps) which expects a single input (and output) """
    def mySingleInputFunct(eps):
        # This will be used for twist, sweep. dihedral, and chord. Returns a
        # single value, as eps is expected to be a single value
        return 5
    wing = LiftingSurface(construct_geometry=False)
    wing.ChordFunct = mySingleInputFunct
    wing.SweepFunct = mySingleInputFunct
    wing.DihedralFunct = mySingleInputFunct
    wing.TwistFunct = mySingleInputFunct
    wing.AirfoilFunct = myAirfoilFunctionAirliner   # Builtin example
    wing.GenerateSectionCurves()
    assert(len(wing._Sections) > 0)


def test_AspectRatio(simple_wing):
    """For a simple straight wing with AR 5 and chord 1, calculate the
    actual aspect ratio and test the output is equal to the expected value"""
    assert(np.abs(simple_wing.CalculateAspectRatio() - 5) < 1e-5)


def test_ProjectedArea(simple_wing):
    """For a simple straight wing with AR 5 and chord 1, calculate the
    projected area and test the output is equal to the expected value"""
    assert(np.abs(simple_wing.CalculateProjectedArea() - 5) < 1e-5)


def test_SemiSpan(simple_wing):
    """For a simple straight wing with AR 5 and chord 1, calculate the
    semi span and test the output is equal to the expected value"""
    assert(np.abs(simple_wing.CalculateSemiSpan() - 5) < 1e-5)


def test_RectangularWing():
    """For a simple straight wing with known area, calculate the projected
    area and test the output is equal to the expected value"""
    NSeg = 1
    wing = LiftingSurface(ChordFunct=SimpleChordFunction,
                          DihedralFunct=SimpleDihedralFunction,
                          SweepFunct=SimpleSweepFunction,
                          AirfoilFunct=SimpleAirfoilFunction,
                          TwistFunct=SimpleTwistFunction,
                          ScaleFactor=5,
                          ChordFactor=0.2,
                          SegmentNo=NSeg)
    # reference values (area and aspect ratio of a 5 by 1 rectangular wing)
    ex_area = 5
    ex_AR = 5
    span = 5
    # Test that the area and calculated aspect ratio are as expected
    tol = 1e-5
    assert(np.abs(wing.LSP_area - ex_area) < tol)
    assert(np.abs(wing.AR - ex_AR) < tol)
    assert(np.abs(wing.ActualSemiSpan - span) < tol)

    # Also test that a multiple segment wing gives the same result
    NSeg = 21
    wing = LiftingSurface(ChordFunct=SimpleChordFunction,
                          DihedralFunct=SimpleDihedralFunction,
                          SweepFunct=SimpleSweepFunction,
                          AirfoilFunct=SimpleAirfoilFunction,
                          TwistFunct=SimpleTwistFunction,
                          ScaleFactor=5,
                          ChordFactor=0.2,
                          SegmentNo=NSeg)
    assert(np.abs(wing.LSP_area - ex_area) < tol)
    assert(np.abs(wing.AR - ex_AR) < tol)
    assert(np.abs(wing.ActualSemiSpan - span) < tol)


def test_SweptWing():
    """For a 15 deg swept wing with known area (unit chord, Aspect Ratio 5),
    calculate the projected area and test the output is equal to the expected
    value"""
    NSeg = 1
    LAMBDA = 15    # uniform sweep, in degrees
    ScaleFactor = 5
    ChordFactor = 0.2
    wing = LiftingSurface(ChordFunct=SimpleChordFunction,
                          DihedralFunct=SimpleDihedralFunction,
                          SweepFunct=(lambda eps: np.ones_like(eps) * 15),
                          AirfoilFunct=SimpleAirfoilFunction,
                          TwistFunct=SimpleTwistFunction,
                          ScaleFactor=ScaleFactor,
                          ChordFactor=ChordFactor,
                          SegmentNo=NSeg)
    # expected area = b * h, where b is the wing chord, h is the semi-span
    LE_length = ScaleFactor     # LE is straight: new length is 1 * ScaleFactor
    span = LE_length * np.cos(np.radians(LAMBDA))
    chord = ChordFactor * ScaleFactor

    ex_area = chord * span
    ex_AR = span / float(chord)

    tol = 1e-5
    assert(np.abs(wing.LSP_area - ex_area) < tol)
    assert(np.abs(wing.AR - ex_AR) < tol)
    assert(np.abs(wing.ActualSemiSpan - span) < tol)


def test_update_ApexPoint():
    """Tests that Build is triggered on updating Apex Point"""
    # Create example airliner wing
    P = (0, 0, 0)
    wing = LiftingSurface(P, mySweepAngleFunctionAirliner,
                          myDihedralFunctionAirliner,
                          myTwistFunctionAirliner,
                          myChordFunctionAirliner,
                          myAirfoilFunctionAirliner)

    # By adding a point to the wing, we will see if the move transformation
    # has been performed when the ApexPoint attribute is changed:
    from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeVertex
    v = BRepBuilderAPI_MakeVertex(gp_Pnt(0, 0, 0)).Vertex()
    wing['test_pnt'] = v

    # Updating the apex should move the surface (and test vertex) through
    # the vector newApex - oldApex
    wing.ApexPoint = gp_Pnt(10, 10, 10)

    # Retrieve the vertex and point from the translated shape
    from OCC.TopoDS import topods_Vertex
    from OCC.BRep import BRep_Tool_Pnt
    vout = topods_Vertex(wing['test_pnt'])
    p = BRep_Tool_Pnt(vout)

    # Check that the vertex was correctly transformed
    xyz = [p.X(), p.Y(), p.Z()]
    assert(xyz == [10, 10, 10])


# def test_Fit_BlendedTipDevice(simple_wing):
#     # Fit a blended winglet to this wing and test the output
#     wing = simple_wing

#     wing.Fit_BlendedTipDevice(rootchord_norm=0.8, spanfraction=0.1, cant=40,
#                              transition=0.1, sweep=40, taper=0.7)

#     # Test the (theoretical) tip chord equals the winglet root chord:
#     assert((Wing.ChordFunct(1) * Wing.ScaleFactor * Wing.ChordFactor) ==
#         Winglet.ChordFunct(0) * Winglet.ScaleFactor * Winglet.ChordFactor)

#     # Test the length of the LE curve is the correct spanfraction