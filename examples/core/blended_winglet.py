# -*- coding: utf-8 -*-
# @Author: pc12g10
# @Date:   2016-08-11 14:19:53
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-08-12 17:38:55

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
    ChordFactor = 0.3
    ScaleFactor = 7.951

    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunction,
                                         myDihedralFunction,
                                         myTwistFunction,
                                         myChordFunction,
                                         myAirfoilFunction,
                                         SegmentNo=NSeg,
                                         ScaleFactor=ScaleFactor,
                                         ChordFactor=ChordFactor)

    Winglet = Wing.Fit_BlendedTipDevice(
        0.8,
        cant=15,
        )

    # Wing.Display(display)
    # Winglet.Display(display)

    # This is just me trying to create a phantom surface:
    shapelist = Wing.values() + Winglet.values()

    # Method 1: Quilting
    # from OCC.BRepTools import BRepTools_Quilt
    # from OCC.TopExp import TopExp_Explorer
    # from OCC.TopAbs import TopAbs_FACE
    # from OCC.TopoDS import topods_Face    
    # quilt = BRepTools_Quilt()
    # for shape in shapelist:
    #     quilt.Add(shape)
    
    # # quilt.Bind()
    # shells = quilt.Shells()
    # display.DisplayShape(shells)

    # import airconics.AirCONICStools as act
    # act.export_STEPFile([shells], 'shells.step')


    # Method 2: Sewing
    from OCC.TopExp import TopExp_Explorer
    from OCC.TopAbs import TopAbs_FACE
    from OCC.TopoDS import topods_Face
    from OCC.BRepBuilderAPI import BRepBuilderAPI_Sewing

    sewing = BRepBuilderAPI_Sewing()
    for shape in shapelist:
        exp = TopExp_Explorer(shape, TopAbs_FACE)
        while exp.More():
            face = topods_Face(exp.Current())
            sewing.Add(face)
            exp.Next()

    sewing.Perform()
    sewed_shape = sewing.SewedShape()
    print(type(sewed_shape))
    # act.export_STEPFile([sewed_shape], 'sewing.step')

    # display.DisplayShape(sewed_shape)
    for section in Wing.Sections:
        display.DisplayShape(section.Curve)
    for section in Winglet.Sections:
        display.DisplayShape(section.Curve)    




    start_display()
