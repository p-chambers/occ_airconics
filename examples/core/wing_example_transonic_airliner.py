# -*- coding: utf-8 -*-
"""
Created on Mon Jan  4 17:28:37 2016

Example script for generating a transonic airliner wing external geometry.

Note: These example functions are also available in the built-in
airconics.examples.wing_example_transonic_airliner module, however they are
repeated here for clarity

@author: pchambers
"""
import numpy as np
from airconics import primitives
from airconics import liftingsurface
from airconics import AirCONICStools as act

# ==============================================================================
# Transonic passanger airliner wing geometry example
# (planform similar to that of the Boeing 787 family)
# ==============================================================================


def myDihedralFunctionAirliner(Epsilon):
    """User-defined function describing the variation of dihedral as a function
    of the leading edge coordinate"""
    BaseDihedral = 7

    # A simple model of a loaded wing shape:
    return BaseDihedral + Epsilon*Epsilon*10


def myTwistFunctionAirliner(Epsilon):
    """User-defined function describing the variation of twist as a function
    of the leading edge coordinate. The coefficients of the polynomial below
    come from the following twist values taken off the CRM (used for the AIAA
    drag prediction workshops):
    Epsilon = 0: twist = 4.24
    Epsilon =0.3: twist = 0.593
    Epsilon = 1: twist = -3.343"""
    return -(6.53*Epsilon*Epsilon - 14.1*Epsilon + 4.24)


def myChordFunctionAirliner(Epsilon):
    """User-defined function describing the variation of chord as a function of
    the leading edge coordinate"""
    ChordLengths = np.array([0.5, 0.3792, 0.2867, 0.232, 0.1763, 0.1393, 0.1155,
                             0.093, 0.0713, 0.055, 0.007])

    EpsArray = np.linspace(0, 1, 11)
    return np.interp(Epsilon, EpsArray, ChordLengths)


def myAirfoilFunctionAirliner(Epsilon, LEPoint, ChordFunct, ChordFactor,
                              DihedralFunct, TwistFunct):
    """Defines the variation of cross section as a function of Epsilon"""
    AfChord = ((ChordFactor*ChordFunct(Epsilon)) /
               np.cos(np.radians(TwistFunct(Epsilon))))
    Af = primitives.Airfoil(LEPoint, ChordLength=AfChord,
                            Rotation=DihedralFunct(Epsilon),
                            Twist=TwistFunct(Epsilon),
                            CRMProfile=True, CRM_Epsilon=Epsilon)
    return Af


def mySweepAngleFunctionAirliner(Epsilon):
    """User-defined function describing the variation of sweep angle as a function
    of the leading edge coordinate"""
    SweepAngles = np.array([90, 87, 35, 35, 35, 35, 35, 35, 35, 35, 80])

    EpsArray = np.linspace(0, 1, 11)

    return np.interp(Epsilon, EpsArray, SweepAngles)


if __name__ == "__main__":
    import airconics
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    # Position of the apex of the wing
    P = (0, 0, 0)

    # Class definition
    NSeg = 10
    ChordFactor = 1
    ScaleFactor = 50

    # Instantiate the class
    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionAirliner,
                                         myDihedralFunctionAirliner,
                                         myTwistFunctionAirliner,
                                         myChordFunctionAirliner,
                                         myAirfoilFunctionAirliner,
                                         SegmentNo=NSeg,
                                         ScaleFactor=ScaleFactor)

    Wing.Display(display)

    for section in Wing._Sections:
        display.DisplayShape(section.Curve, update=True)

#    To export the STEP file, uncomment the following line:
#    Wing.Write('Wing.stp')
#    act.export_STEPFile([Wing['Surface']], 'wing.stp')

    Wing.Write('wing.stp')

    display.DisplayShape(Wing.LECurve)

    start_display()
