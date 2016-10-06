# -*- coding: utf-8 -*-
# @Author: pc12g10
# @Date:   2016-08-11 14:19:53
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-10-06 15:51:08

import numpy as np
from airconics.primitives import Airfoil
from airconics import liftingsurface
import airconics.AirCONICStools as act
# ==============================================================================
# Essentially the AirCONICS jetstream wing with the end cut back and no loading
# (uniform dihedral), and a blended winglet added.
# ==============================================================================


def myDihedralFunction(Epsilon):
    """User-defined function describing the variation of dihedral as a function
    of the leading edge coordinate"""
    return 7


def myTwistFunction(Epsilon):
    """User-defined function describing the variation of twist as a function
    of the leading edge coordinate."""
    RootTwist = 0
    TipTwist  = -2
    return RootTwist + Epsilon*TipTwist


myChordFunction = act.Generate_InterpFunction([1, 0.33], [0,1])


@liftingsurface.airfoilfunct
def myAirfoilFunction(eps):
    af_root = Airfoil(SeligProfile='naca63a418')
    af_tip = Airfoil(SeligProfile='naca63a412')

    profile_dict = {'InterpProfile': True, 'Epsilon': eps, 'Af1': af_root,
                    'Af2': af_tip, 'Eps1': 0, 'Eps2': 1}
    return profile_dict


def mySweepAngleFunction(Epsilon):
    # User-defined function describing the variation of sweep angle as a function
    # of the leading edge coordinate
    return 3


if __name__ == "__main__":
    import airconics
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    # Position of the apex of the wing
    P = (0, 0, 0)

    # Class definition
    NSeg = 1

    # Instantiate the class
    ChordFactor = 0.6
    ScaleFactor = 7.951

    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunction,
                                         myDihedralFunction,
                                         myTwistFunction,
                                         myChordFunction,
                                         myAirfoilFunction,
                                         SegmentNo=NSeg,
                                         ScaleFactor=ScaleFactor,
                                         ChordFactor=ChordFactor)

    Winglet = Wing.Fit_BlendedTipDevice(rootchord_norm=0.8, spanfraction=0.15,
        cant=15,
        transition=0.4
        )

    Wing.Display(display)

    Winglet.Display(display)

    # Why not also try a C wing made from two recursively added winglets?
    Winglet2 = Winglet.Fit_BlendedTipDevice(rootchord_norm=0.8, spanfraction=1,
        transition=0.35, cant=-90, sweep=20, taper=0.8)

    Winglet2.Display(display)

    from airconics.base import AirconicsCollection

    geometry = AirconicsCollection(parts={'Wing': Wing, 'Winglet':Winglet, 'CWinglet':Winglet2})

    # geometry.Write('C_Wing.step')

    start_display()

