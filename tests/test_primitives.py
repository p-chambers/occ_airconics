# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 14:23:37 2016

Tests for airconics airfoil primitives.

@author: pchambers
"""
import pytest
from airconics.primitives import Airfoil
import numpy as np
import OCC.Geom


@pytest.fixture(params=[
    # tuple with {Profile type, Argument Value}
    ('Naca4Profile', '0012'),
    ('SeligProfile', 'b707a'),
    ('CRM_Epsilon', 0.5)
    ])
def airfoil_examples(request):
    return request.param


def test_Null_Airfoil():
    # Generate Empty Airfoil
    Af = Airfoil()
    assert(Af.LE == [0., 0., 0.])
    assert(Af.Rotation == 0)
    assert(Af.Twist == 0)
    assert(Af.ChordLength == 1)
    assert(Af.Profile is None)
    assert(Af.Curve is None)


def test_nonzero_LE():
    # Use Start Point of untwisted, unrotated, unit chord, LE in origin NACA4
    # Airfoil as reference
    default_SP = np.array([1.,  0.,  0.00126])

    # Non-Zero leading edge:
    Af = Airfoil(LeadingEdgePoint=[1., 1., 1.], ChordLength=1,
                 Naca4Profile='0012')
    start_pt = get_Airfoil_startpoint(Af)
    SP_ref = default_SP + np.array([1., 1., 1.])
    assert(np.all((start_pt - SP_ref) < 1e-10))


def test_nonzero_twist():
    # Use Start Point of untwisted, unrotated, unit chord, LE in origin NACA4
    # Airfoil as reference
    default_SP = np.array([1., 0., 0.00126])

    # Test Non-zero Twist (around Y axis):
    Af = Airfoil(ChordLength=1, Naca4Profile='0012', Twist=10)
    start_pt = get_Airfoil_startpoint(Af)

    beta = Af.Twist * np.pi/180.
    SP_ref = twist_matrix(-beta).dot(default_SP)
    assert(np.all((start_pt - SP_ref) < 1e-10))


def test_nonzero_rotation():
    # Use Start Point of untwisted, unrotated, unit chord, LE in origin NACA4
    # Airfoil as reference
    default_SP = np.array([1.,  0.,  0.00126])

    # Test Non-Zero Rotation (Around X axis):
    Af = Airfoil(ChordLength=1, Naca4Profile='0012', Rotation=10)
    start_pt = get_Airfoil_startpoint(Af)
    # Miscellaneous function for creating rotation matrix of th around X axis
    gamma = Af.Rotation * np.pi/180.
    SP_ref = roll_matrix(gamma).dot(default_SP)
    assert(np.all((start_pt - SP_ref) < 1e-10))


def test_Transform_Airfoil():
    # Use Start Point of untwisted, unrotated, unit chord, LE in origin NACA4
    # Airfoil as reference
    default_SP = np.array([1.,  0.,  0.00126])

    # Test Translation, Rotation and Twist :
    Af = Airfoil(LeadingEdgePoint=[1., 1., 1.], ChordLength=1,
                 Naca4Profile='0012', Rotation=10, Twist=5)
    start_pt = get_Airfoil_startpoint(Af)
    beta = Af.Twist * np.pi/180.
    gamma = Af.Rotation * np.pi/180.
    Yrot_mat = twist_matrix(-beta)
    Xrot_mat = roll_matrix(gamma)
    rotation = Yrot_mat.dot(Xrot_mat)
    SP_ref = rotation.dot(default_SP) + np.array([1., 1., 1.])
    assert(np.all((start_pt - SP_ref) < 1e-10))


@pytest.mark.xfail
def test_EnforceSharpTE():
    # Test if airfoil is closed, NACA0012? - Haven't got this feature to work
    raise NotImplementedError


def test_overdefined_Airfoil():
    # Test that two or more specified profiles raises an error
    with pytest.raises(AssertionError):
        Af = Airfoil(SeligProfile='b707a', Naca4Profile='0012')


def test_Airfoil_constructor(airfoil_examples):
    # Unwrap parametrised arguments from the pytest.fixture
    ProfileType, Profile = airfoil_examples

    # Have to use eval here as keyword given in airfoil_examples is a string
    Af = eval("Airfoil(" + ProfileType + "='" + str(Profile) + "')")

    # Test the Airfoil.Profile is the expected dictionary
    assert(ProfileType in Af.Profile.keys())
    assert(Af.Profile[ProfileType] == str(Profile))

    # Test the Curve has been generated
    assert(Af.Curve is not None)

    # Test the python-occ spline object:
    assert(type(Af.Curve) == OCC.Geom.Handle_Geom_BSplineCurve)
    assert(Af.Curve.IsNull() == False)

    # Check the Trailing Edge X is roughly equal to the chord length
    start_pt = get_Airfoil_startpoint(Af)
    assert((np.abs(start_pt[0] - Af.ChordLength) / Af.ChordLength) < 0.01)


def test_Airfoil_manual(airfoil_examples):
    # Unwrap parametrised arguments from the pytest.fixture
    ProfileType, Profile = airfoil_examples

    Af = Airfoil()
    # This is probably a dirty way of doing this, but couldn't think of another
    if ProfileType == 'Naca4Profile':
        Af.AddNACA4(Profile)
    elif ProfileType == 'SeligProfile':
        Af.AddAirfoilFromSeligFile(Profile)
    elif ProfileType == 'CRM_Epsilon':
        Af.AddCRMLinear(Profile)
    else:
        raise NotImplementedError(
            '{} is not an Implemented Airfoil type'.format(ProfileType))

    # Test the Airfoil.Profile is the expected dictionary
    assert(ProfileType in Af.Profile.keys())
    assert(Af.Profile[ProfileType] == str(Profile))

    # Test the Curve has been generated
    assert(Af.Curve is not None)

    # Test the python-occ spline object:
    assert(type(Af.Curve) == OCC.Geom.Handle_Geom_BSplineCurve)
    assert(Af.Curve.IsNull() == False)

    # Check the Trailing Edge X is roughly equal to the chord length
    start_pt = get_Airfoil_startpoint(Af)
    assert((np.abs(start_pt[0] - Af.ChordLength) / Af.ChordLength) < 0.01)


def test_Airfoil_emptystring():
    # Test that an empty profile string creates an 'empty' airfoil
    Af = Airfoil(Naca4Profile='')
    assert(not Af.Curve)

    # Also check Selig airfoils:
    Af = Airfoil(SeligProfile='')
    assert(not Af.Curve)


def test_NACA4_Airfoil_Assertions():
    # Test that a non-string raises a type error
    with pytest.raises(TypeError):
        Af = Airfoil(Naca4Profile=100)
    # Test that a non existant Selig Airfoil raises an error
    with pytest.raises(AssertionError):
        Af = Airfoil(Naca4Profile='SomeImaginaryAirfoil')


def test_CRM_Airfoil_Assertions():
    # Test that an out of range epsilon produces an error
    with pytest.raises(AssertionError):
        Af = Airfoil(CRM_Epsilon=10)
    # Test that a string containing letters con raises a type error
    with pytest.raises(ValueError):
        Af = Airfoil(CRM_Epsilon='d')


# def test_NACA5_Airfoil_Assertions
#


def test_Selig_Airfoil_Assertions():
    # Test that a non-string raises a type error
    with pytest.raises(TypeError):
        Af = Airfoil(SeligProfile=100)
    # Test that a non existant Selig Airfoil raises an error
    with pytest.raises(AssertionError):
        Af = Airfoil(SeligProfile='SomeImaginaryAirfoil')


# END OF TESTS

# ---------------------------------------------------------------------------
# Misc Functions:
def get_Airfoil_startpoint(Af):
    curve = Af.Curve.GetObject()
    StartPoint = curve.StartPoint()
    return np.array([StartPoint.X(), StartPoint.Y(), StartPoint.Z()])


def twist_matrix(beta):
    """Return the rotation matrix of a 3d rotation around X axis by angle beta:
    beta in radians, anticlockwise"""
    return np.array([[np.cos(beta),  0., np.sin(beta)],
                     [0.,            1.,           0.],
                     [-np.sin(beta), 0., np.cos(beta)]])


def roll_matrix(gamma):
    """Return the rotation matrix of a 3d rotation around Y axis by angle beta:
    beta in radians, anticlockwise"""
    return np.array([[1.,            0.,             0.],
                     [0., np.cos(gamma), -np.sin(gamma)],
                     [0., np.sin(gamma),  np.cos(gamma)]])
# ---------------------------------------------------------------------------
