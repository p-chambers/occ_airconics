# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 11:39:16 2016

Example script for generating the lifting surfaces for the tail of a
transport aircraft (fin and tailplane external geometry). This uses built-in
tailplane functions from airconics.examples: See
wing_example_transonic_airliner.py for how these functions are implemented

# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

@author: pchambers
"""
if __name__ == '__main__':
    from airconics.examples.tailplane_example_transonic_airliner import *
    from airconics import liftingsurface
    import airconics.AirCONICStools as act

    # Import x axis to rotate fin around
    from OCC.gp import gp_Ax1, gp_Pnt, gp_Dir
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    # Position of the apex of the fin
    P = [36.98-0.49-0.02, 0.0, 2.395-0.141]

    SegmentNo = 100
    ChordFact = 1.01
    ScaleFact = 21.93

    Fin = liftingsurface.LiftingSurface(P, mySweepAngleFunctionFin,
                                        myDihedralFunctionFin,
                                        myTwistFunctionFin,
                                        myChordFunctionFin,
                                        myAirfoilFunctionFin,
                                        SegmentNo=SegmentNo,
                                        ChordFactor=ChordFact,
                                        ScaleFactor=ScaleFact)

#    Create the rotation axis centered at the apex point in the x direction
    RotAxis = gp_Ax1(gp_Pnt(*P), gp_Dir(1, 0, 0))

    # Having some problem with the fin loft: display some airfoils
    # to figure out what's going on:
#    for section in Fin._Sections:
#        curve = section.Curve.GetObject()
#        curve.Scale(gp_Pnt(0., 0., 0.), ScaleFact)
#        display.DisplayShape(section.Curve, update=True)

    Fin.Rotate(RotAxis, 90)
    display.DisplayShape(Fin.Shape, update=True)

    # Position of the apex of the tailplane
    P = [43, 0.000, 1.633+0.02]

    SegmentNo = 100
    ChordFactor = 1.01
    ScaleFactor = 17.3

    TP = liftingsurface.LiftingSurface(P, mySweepAngleFunctionTP,
                                       myDihedralFunctionTP,
                                       myTwistFunctionTP,
                                       myChordFunctionTP,
                                       myAirfoilFunctionTP,
                                       SegmentNo=SegmentNo,
                                       ChordFactor=ChordFact,
                                       ScaleFactor=ScaleFact)

    display.DisplayShape(TP.Shape, update=True)

    TP2 = act.mirror(TP.Shape, plane='xz', copy=True)

    # Note: TP2 is a TopoDS_Shape, not a wing and DisplayShape is called as:
    display.DisplayShape(TP2, update=True)

    start_display()
