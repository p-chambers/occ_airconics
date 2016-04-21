# -*- coding: utf-8 -*-
"""
Created on Fri Mar 11 11:36:51 2016

@author: pchambers
"""
import numpy as np
from airconics import primitives


def myDihedralFunctionBoxWing(Epsilon):
    """User-defined function describing the variation of dihedral as a function
    of the leading edge coordinate
    Notes
    -----
    Could also use numpy vectorize in this function
    """
    D1 = 0
    D2 = 180
    Transition1 = 0.45
    Transition2 = 0.55

    # Known values of dihedral at start, transition and end locations
    EpsArray = np.array([0, Transition1, Transition2, 1])
    DihedralArray = np.array([D1, D1, D2, D2])

    return np.interp(Epsilon, EpsArray, DihedralArray)


def myTwistFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of twist as a function
    # of the leading edge coordinate
    RootTwist = 0
    TipTwist  = 0
    return RootTwist + Epsilon*TipTwist


def myChordFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of chord as a function of
    # the leading edge coordinate
    return 1


def myAirfoilFunctionBoxWing(Epsilon, LEPoint, ChordFunct, ChordFactor,
                             DihedralFunct, TwistFunct):
    # Defines the variation of cross section as a function of Epsilon

    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon)) / \
        np.cos(np.radians(TwistFunct(Epsilon)))

    SmoothingPasses = 1

    Camber1 = 5.0
    Camber2 = -5.0
    Transition1 = 0.45
    Transition2 = 0.55

    # Known values of dihedral at start, transition and end locations
    EpsArray = np.array([0, Transition1, Transition2, 1])
    CamberArray = np.array([Camber1, Camber1, Camber2, Camber2])

    Camber = np.interp(Epsilon, EpsArray, CamberArray)
    prof = str(int(round(Camber))) + '310'

    Af = primitives.Airfoil(LEPoint,
                            AirfoilChordLength,
                            DihedralFunct(Epsilon),
                            TwistFunct(Epsilon),
                            Naca4Profile=prof)
    return Af


def mySweepAngleFunctionBoxWing(Epsilon):
    # User-defined function describing the variation of sweep angle as a
    # function of the leading edge coordinate

    S1 = 25
    S2 = -25
    Boundary1 = 0.45
    Boundary2 = 0.55

    # Known values of dihedral at start, transition and end locations
    EpsArray = np.array([0, Boundary1, Boundary2, 1])
    SweepArray = np.array([S1, S1, S2, S2])

    return np.interp(Epsilon, EpsArray, SweepArray)

if __name__ == '__main__':
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    from airconics import liftingsurface as LS

    P = (0, 0, 0)
    LooseSurf = 1
    SegmentNo = 101

    ChordFactor = 0.1
    ScaleFactor = 20
    # The wing tip is turned off, as a box wing has no exposed tip
    Wing = LS.LiftingSurface(P, mySweepAngleFunctionBoxWing,
                             myDihedralFunctionBoxWing,
                             myTwistFunctionBoxWing,
                             myChordFunctionBoxWing,
                             myAirfoilFunctionBoxWing,
                             SegmentNo=SegmentNo,
                             ChordFactor=ChordFactor,
                             ScaleFactor=ScaleFactor)

    Wing.Display(display)

    start_display()
