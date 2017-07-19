# -*- coding: utf-8 -*-
"""
Created on Fri Jun 24 22:21:02 2016

@author: paul
"""
from airconics import primitives, AirCONICStools as act
from airconics.liftingsurface import LiftingSurface
from airconics.engine import Engine
import airconics.examples.wing_example_transonic_airliner as wingex


if __name__ == "__main__":
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

#    Generate a wing first to attach the engine to
    P = (0, 0, 0)

    SegmentNo = 10
    ChordFactor = 1
    ScaleFactor = 44.56
    Wing = LiftingSurface(P, wingex.mySweepAngleFunctionAirliner,
                          wingex.myDihedralFunctionAirliner,
                          wingex.myTwistFunctionAirliner,
                          wingex.myChordFunctionAirliner,
                          wingex.myAirfoilFunctionAirliner,
                          SegmentNo=SegmentNo,
                          ChordFactor=ChordFactor,
                          ScaleFactor=ScaleFactor)

    Wing.Display(display)
    SpanStation = 0.3              # The engine is to be placed at 30% span
    EngineDia = 2.9
    NacelleLength = 1.95*EngineDia

    EngineSection, HChord = act.CutSect(Wing['Surface'], SpanStation)
    Chord = HChord.GetObject()
    CEP = Chord.EndPoint()
    display.DisplayShape(Chord, update=True, color='black')
    # Variables controlling the position of the engine with respect to the wing
    EngineCtrFwdOfLE = 0.98
    EngineCtrBelowLE = 0.35
    Scarf_deg = 4
    Centreloc = [CEP.X()-EngineCtrFwdOfLE*NacelleLength,
                 CEP.Y(),
                 CEP.Z()-EngineCtrBelowLE*NacelleLength]

#   Now build the engine and its pylon
    eng1 = Engine(HChord,
                  CentreLocation=Centreloc,
                  ScarfAngle=Scarf_deg,
                  HighlightRadius=EngineDia/2.0,
                  MeanNacelleLength=NacelleLength)

#    Display Engine Components:
    eng1.Display(display)

    eng1.DisplayBBox(display)


    start_display()
