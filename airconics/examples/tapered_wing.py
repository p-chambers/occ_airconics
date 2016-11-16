# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-11-16 14:14:52
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-11-16 14:34:16
import numpy as np
from airconics.primitives import Airfoil
from airconics.liftingsurface import LiftingSurface, airfoilfunct
import airconics.AirCONICStools as act

# ==============================================================================
# Essentially the AirCONICS jetstream wing with the end cut back and no loading
# (uniform dihedral), and a blended winglet added.
# ==============================================================================


def TaperedWingDihedralFunction(Epsilon):
    """User-defined function describing the variation of dihedral as a function
    of the leading edge coordinate"""
    return 7


def TaperedWingTwistFunction(Epsilon):
    """User-defined function describing the variation of twist as a function
    of the leading edge coordinate."""
    RootTwist = 0
    TipTwist = -2
    return RootTwist + Epsilon * TipTwist


TaperedWingChordFunction = act.Generate_InterpFunction([1, 0.33], [0, 1])


@airfoilfunct
def TaperedWingAirfoilFunction(eps):
    af_root = Airfoil(SeligProfile='naca63a418')
    af_tip = Airfoil(SeligProfile='naca63a412')

    profile_dict = {'InterpProfile': True, 'Epsilon': eps, 'Af1': af_root,
                    'Af2': af_tip, 'Eps1': 0, 'Eps2': 1}
    return profile_dict


def TaperedWingSweepFunction(Epsilon):
    """User-defined function describing the variation of sweep angle as a function
    of the leading edge coordinate"""
    return np.ones_like(Epsilon) * 3


if __name__ == "__main__":
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    # Position of the apex of the wing
    P = (0, 0, 0)

    # Class definition
    NSeg = 10

    # Instantiate the class
    ChordFactor = 0.283
    ScaleFactor = 50

    Wing = LiftingSurface(P, TaperedWingSweepFunction,
                          TaperedWingDihedralFunction,
                          TaperedWingTwistFunction,
                          TaperedWingChordFunction,
                          TaperedWingAirfoilFunction,
                          SegmentNo=NSeg,
                          ScaleFactor=ScaleFactor, 
                          ChordFactor=ChordFactor)

    Wing.Display(display)

    start_display()
