# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-07-26 17:37:52
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-09-30 12:23:54
import numpy as np
from airconics import primitives
from airconics import liftingsurface


def SimpleDihedralFunction(Epsilon):
    """User-defined function describing the variation of dihedral as a function
    of the leading edge coordinate"""
    return np.ones_like(Epsilon)


def SimpleTwistFunction(Epsilon):
    """User-defined function describing the variation of twist as a function
    of the leading edge coordinate. The coefficients of the polynomial below
    come from the following twist values taken off the CRM (used for the AIAA
    drag prediction workshops):
    Epsilon = 0: twist = 4.24
    Epsilon =0.3: twist = 0.593
    Epsilon = 1: twist = -3.343"""
    return np.ones_like(Epsilon)



def SimpleChordFunction(Epsilon):
    """User-defined function describing the variation of chord as a function of
    the leading edge coordinate"""
    return 1



def SimpleAirfoilFunction(Epsilon, LEPoint, ChordFunct, ChordFactor,
                              DihedralFunct, TwistFunct):
    """constant cross section with Epsilon"""
    AfChord = ChordFunct(Epsilon) * ChordFactor
    Af = primitives.Airfoil(LEPoint, ChordLength=AfChord,
                            Rotation=DihedralFunct(Epsilon),
                            Twist=TwistFunct(Epsilon),
                            Naca4Profile='0012')
    return Af


def SimpleSweepFunction(Epsilon):
    """User-defined function describing the variation of sweep angle as a function
    of the leading edge coordinate"""
    return np.ones_like(Epsilon)


if __name__ == "__main__":
    import airconics
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    # Position of the apex of the wing
    P = (0,0,0)
    
    # Class definition
    NSeg = 10
    
    # Instantiate the class
    ChordFactor = 1
    ScaleFactor = 50
    
    # First try a standard CRM airfoil:
    # Af_crm = airconics.primitives.Airfoil([0., 6., 1.], CRMProfile=True, CRM_Epsilon=0.8)
    # display.DisplayShape(Af_crm.Curve, update=True, color='GREEN');
    
    Wing = liftingsurface.LiftingSurface(P, SimpleSweepFunction, 
        SimpleDihedralFunction, 
        SimpleTwistFunction, 
        SimpleChordFunction, 
        SimpleAirfoilFunction, SegmentNo=NSeg, ScaleFactor=ScaleFactor)

    Wing.Display(display)

    start_display()
