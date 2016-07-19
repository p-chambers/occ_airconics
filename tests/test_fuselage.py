# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-07-14 16:38:17
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-07-19 16:03:37
import pytest
from airconics.fuselage_oml import Fuselage
import airconics.AirCONICStools as act
import numpy as np
import cPickle
import os


@pytest.fixture
def empty_fuselage():
    """Initialises an empty fuselage class such that its functions can be
    tested later"""
    fus = Fuselage(construct_geometry=False)
    return fus


# def assert_equal_curves(curve1, curve2):
    # """Helper function: determines if two bezier curves are equal"""
    # assert(curve1.NbPoles() == curve2.NbPoles())

    # for i in range(curve1.NbPoles()):
    #     assert(curve1.Pole(i).IsEqual(curve.Pole(i)))


def test_empty_fuselage():
    # Do nothing, simply check if this works:
    fus = Fuselage(construct_geometry=False)


def test_AirlinerFuselagePlanView(empty_fuselage):
    """Tests the Airliner plan curve control point generator function"""
    fuselage = empty_fuselage

    AFPVPort, NoseEndX, TailStartX = \
        fuselage.AirlinerFuselagePlanView(fuselage.NoseLengthRatio,
                                          fuselage.TailLengthRatio)

    AFPVPort_expect = np.array([[0., 0., 0.],
                                [0., -0.2541, 0.],
                                [0.843612, -1.003695, 0.],
                                [3.17625, -2.05821, 0.],
                                [6.395697, -2.729034, 0.],
                                [10.164, -2.92215, 0.],
                                [10.164, -2.92215, 0.],
                                [39.51255, -2.92215, 0.],
                                [39.51255, -2.92215, 0.],
                                [41.743548, -2.861166, 0.],
                                [51.6742842, -1.227303, 0.],
                                [55.902, -0.2507967, 0.]])

    assert(np.allclose(AFPVPort, AFPVPort_expect)),\
        'Fuselage plan view curve control points are different from expected'

    NoseEndX_expect, TailStartX_expect = (10.164, 39.51255)

    # Tests if the end of the nose section is the expected default
    assert(np.abs(NoseEndX - NoseEndX_expect) < 1e-8)

    # Tests if the start of the tail section is the expected default
    assert(np.abs(TailStartX - TailStartX_expect) < 1e-8)


def test_AirlinerFuselageSideView(empty_fuselage):
    """Tests the Airliner side view curve control point generator function"""
    fuselage = empty_fuselage
    FSVU, FSVL = fuselage.AirlinerFuselageSideView(fuselage.NoseLengthRatio,
                                                   fuselage.TailLengthRatio)

    FSVU_expect = np.array([[0., 0., 0],
                            [0., 0., 0.7623],
                            [3.544695, 0., 3.930927],
                            [10.164, 0., 4.284126],
                            [10.164, 0., 4.284126],
                            [39.51255, 0., 4.284126],
                            [39.51255, 0., 4.284126],
                            [48.774495, 0., 3.936009],
                            [55.902, 0., 2.297064]])
    FSVL_expect = np.array([[0., 0., 0.],
                            [0., 0., -0.7623],
                            [2.406327, 0., -1.313697],
                            [10.164, 0., -1.661814],
                            [10.164, 0., -1.661814],
                            [39.51255, 0., -1.661814],
                            [39.51255, 0., -1.661814],
                            [47.737767, 0., -0.650496],
                            [55.902, 0., 1.763454]])

    assert(np.allclose(FSVL, FSVL_expect)),\
        'Side view lower curve control points are different from expected'
    assert(np.allclose(FSVU, FSVU_expect)),\
        'Side view upper curve control points are different from expected'


def test_FuselageLongitudinalGuideCurves(empty_fuselage):
    """Tests the Airliner Longitudinal guide curve control point generator
    function

    Note: Curves are not picklable, otherwise this function could have
    unpickled a reference curve for testing
    """
    # This doesn't work : Load the pickled list of bspline curves
    # curve_path = os.path.join('.', 'data', 'Fuselage_guides.p')
    # guide_list = cPickle.load(open(curve_path, 'rb'))
    # Upper_ex, Port_ex, Lower_ex, Starboard_ex = guide_list

    # Create the Fuselage guide curves. If this fails, debug this function
    fuselage = empty_fuselage
    HStarboardCurve, HPortCurve, UpperCurve, LowerCurve, FSVMeanCurve,\
        NoseEndX, TailStartX, EndX = fuselage.FuselageLongitudinalGuideCurves(
            fuselage.NoseLengthRatio, fuselage.TailLengthRatio)

    StarboardCurve = HStarboardCurve.GetObject()
    PortCurve = HPortCurve.GetObject()

    # Could check the curves against a reference here, but this would involve
    # enforcing a particular shape solution. For now, assume that if this
    # function completes, then valid guides have been found


def test_BuildFuselageOML(empty_fuselage):
    """Test build fuselage method from initialised empty object, using
    default values
    """
    fuselage = empty_fuselage
    fuselage.BuildFuselageOML()

    # Check the outer mould line has been produced and exists in the dict
    assert('OML' in fuselage)


def test_Fuselage_full():
    """Build fuselage with the default parameters"""
    fus = Fuselage()
    assert('OML' in fus)


def test_TransformOML(empty_fuselage):
    """tests if the transform OML function works

    This function wraps base class method 'TransformComponents_Nonuniformal,
    therefore no great depth is needed here"""
    fuselage = empty_fuselage
    fuselage.BuildFuselageOML()

    # If errors arise here, check base class TransformComponents_Nonuniformal
    fuselage.TransformOML()


@pytest.mark.xfail
def test_WindowContour(empty_fuselage):
    """Uses the empty fuselage to test the window contour method gives the
    expected result"""
    raise(NotImplementedError)


@pytest.mark.xfail
def test_MakeWindow():
    raise(NotImplementedError)


@pytest.mark.xfail
def test_CockpitwindowContours():
    raise(NotImplementedError)
