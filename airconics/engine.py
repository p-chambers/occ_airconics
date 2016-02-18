# -*- coding: utf-8 -*-
"""
Created on Wed Feb 10 16:38:47 2016

This is a work in progress

@author: pchambers
"""

from airconics import primitives, AirCONICStools as act
import numpy as np
from airconics.examples.wing_example_transonic_airliner import *
from OCC.GC import GC_MakeCircle
from OCC.gp import gp_Pnt, gp_Vec
from OCC.Geom import Handle_Geom_Circle


# Will add a base class for rotating engine etc. later for inheritance
class Engine:
    def __init__(self,
                 EngineSection,
                 Chord, 
                 CentreLocation=[0,0,0],
                 ScarfAngle=3,
                 HighlightRadius=1.45,
                 MeanNacelleLength=5.67):
        """Constructor for the engine class"""
        self.EngineSection = EngineSection
        self.Chord = Chord
        self.CentreLocation = CentreLocation
        self.ScarfAngle = ScarfAngle
        self.HighlightRadius = HighlightRadius
        self.MeanNacelleLength = MeanNacelleLength
        self.Build_Engine()
        
    def Build_Engine(self):
        self.Engine, self.Pylon = self._TurbofanNacelle()
        return None
    
    def _TurbofanNacelle(self):
        """The defaults yield a nacelle similar to that of an RR Trent 1000 / GEnx
        """
        MeanNacelleLength = self.MeanNacelleLength
        HighlightRadius = self.HighlightRadius
        HighlightDepth = 0.12 * self.MeanNacelleLength
        SectionNo = 100
        
        # Draw the nacelle with the centre of the intake highlight circle in 0,0,0
        Highlight = GC_MakeCircle(gp_Pnt(0,0,HighlightRadius),
                                  gp_Pnt(0,-HighlightRadius,0),
                                  gp_Pnt(0,0,-HighlightRadius)).Value().GetObject()
        HighlightCutterCircle = GC_MakeCircle(gp_Pnt(0,0,HighlightRadius*1.5),
                                              gp_Pnt(0,-HighlightRadius*1.5,0),
                                              gp_Pnt(0,0,-HighlightRadius*1.5)).Value().GetObject()
        # Fan disk for CFD boundary conditions
        FanCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength*0.25, 0, 0)))
        wire = act.make_wire(act.make_edge(FanCircle))
        FanDisk = act.make_face(wire)
        self._FC = FanCircle
        self._FDisk = FanDisk
        
#        # Aft outflow for CFD boundary conditions
        BypassCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength*0.85, 0, 0)))
        wire = act.make_wire(act.make_edge(BypassCircle))
        BypassDisk = act.make_face(wire)#        rs.DeleteObjects([FanCircle, BypassCircle])
#    
#        # Outflow cone
        TailConeBasePoint = np.array([MeanNacelleLength*0.84, 0,0])
        TailConeHeight    = MeanNacelleLength*1.35
        TailConeRadius    =  HighlightRadius*0.782
        TailCone = act.AddCone(TailConeBasePoint, TailConeRadius, TailConeHeight)
        self._TailCone = TailCone
#        # Spinner cone
#        SpinnerConeBasePoint = [MeanNacelleLength*0.26, 0,0]
#        SpinnerConeApex    = [MeanNacelleLength*0.08, 0, 0]
#        SpinnerConeRadius    =  MeanNacelleLength*0.09
#        Spinner = rs.AddCone(SpinnerConeBasePoint, SpinnerConeApex, SpinnerConeRadius)
#    
#        
#        # Tilt the intake
#        RotVec = rs.VectorCreate((0,0,0),(0,1,0))
#        Highlight = rs.RotateObject(Highlight, (0,0,0), ScarfAngle, axis = RotVec)
#        
#        # Set up the disk for separating the intake lip later
#        HighlightCutterCircle = rs.RotateObject(HighlightCutterCircle, (0,0,0), ScarfAngle, axis = RotVec)
#        HighlightCutterDisk = rs.AddPlanarSrf(HighlightCutterCircle)
#        rs.DeleteObject(HighlightCutterCircle)
#        rs.MoveObject(HighlightCutterDisk, (HighlightDepth, 0,0))
#        
#        # Build the actual airfoil sections to define the nacelle
#        HighlightPointVector = rs.DivideCurve(Highlight, SectionNo)
#        
#        Sections = []
#        TailPoints = []
#        Rotation = 0
#        Twist = 0
#        AirfoilSeligName = 'goe613'
#        SmoothingPasses = 1
#    
#        for HighlightPoint in HighlightPointVector:
#            ChordLength = MeanNacelleLength - HighlightPoint.X
#            Af = primitives.Airfoil(HighlightPoint,ChordLength, Rotation, Twist, SeligPath=airconics_setup.SeligPath)
#            AfCurve,Chrd = primitives.Airfoil.AddAirfoilFromSeligFile(Af, AirfoilSeligName, SmoothingPasses)
#            rs.DeleteObject(Chrd)
#            P = rs.CurveEndPoint(AfCurve)
#            list.append(TailPoints, P)
#            AfCurve = act.AddTEtoOpenAirfoil(AfCurve)
#            list.append(Sections, AfCurve)
#            Rotation = Rotation + 360.0/SectionNo
#        
#        list.append(TailPoints, TailPoints[0])
#        
#        # Build the actual nacelle OML surface
#        EndCircle = rs.AddInterpCurve(TailPoints)
#        Nacelle = rs.AddSweep2([Highlight, EndCircle], Sections, closed = True)
#        # Separate the lip
#        Cowling, HighlightSection = rs.SplitBrep(Nacelle, HighlightCutterDisk, True)
#        
#        
#        # Now build the pylon between the engine and the specified chord on the wing
#        CP1 = [MeanNacelleLength*0.26+CentreLocation[0],CentreLocation[1],CentreLocation[2]+HighlightRadius*0.1]
#        CP2 = [MeanNacelleLength*0.4+CentreLocation[0],CentreLocation[1],HighlightRadius*1.45+CentreLocation[2]]
#        CP3 = rs.CurveEndPoint(Chord)
#        rs.ReverseCurve(Chord)
#        CP4 = rs.CurveEndPoint(Chord)
#    
#        # Move the engine into its actual place on the wing
#        rs.MoveObjects([HighlightSection, Cowling, FanDisk, BypassDisk, TailCone, Spinner], CentreLocation)
#    
#        # Pylon wireframe
#        PylonTop = rs.AddInterpCurve([CP1, CP2, CP3, CP4])
#        PylonAf = primitives.Airfoil(CP1,MeanNacelleLength*1.35, 90, 0, airconics_setup.SeligPath)
#        PylonAfCurve,PylonChord = primitives.Airfoil.AddNACA4(PylonAf, 0, 0, 12, 3)
#        LowerTE = rs.CurveEndPoint(PylonChord)
#        PylonTE = rs.AddLine(LowerTE, CP4) 
#    
#        # Create the actual pylon surface
#        PylonLeft = rs.AddNetworkSrf([PylonTop, PylonAfCurve, PylonTE])
#        rs.MoveObject(PylonLeft, (0,-CentreLocation[1],0))
#        PylonRight = act.MirrorObjectXZ(PylonLeft)
#        rs.MoveObject(PylonLeft,  (0,CentreLocation[1],0))
#        rs.MoveObject(PylonRight, (0,CentreLocation[1],0))
#        PylonAfCurve = act.AddTEtoOpenAirfoil(PylonAfCurve)
#        PylonAfSrf = rs.AddPlanarSrf(PylonAfCurve)
#    
#        # Assigning basic surface properties
#        act.AssignMaterial(Cowling, "ShinyBABlueMetal")
#        act.AssignMaterial(HighlightSection, "UnpaintedMetal")
#        act.AssignMaterial(TailCone, "UnpaintedMetal")
#        act.AssignMaterial(FanDisk, "FanDisk")
#        act.AssignMaterial(Spinner, "ShinyBlack")
#        act.AssignMaterial(BypassDisk, "FanDisk")
#        act.AssignMaterial(PylonLeft,"White_composite_external")
#        act.AssignMaterial(PylonRight,"White_composite_external")
#    
#        # Clean-up
#        rs.DeleteObject(HighlightCutterDisk)
#        rs.DeleteObjects(Sections)
#        rs.DeleteObject(EndCircle)
#        rs.DeleteObject(Highlight)
#        rs.DeleteObjects([PylonTop, PylonAfCurve, PylonChord, PylonTE])
#        
#        
#        rs.Redraw()
#        
#        TFEngine = [Cowling, HighlightSection, TailCone, FanDisk, Spinner, BypassDisk]
#        TFPylon = [PylonLeft, PylonRight, PylonAfSrf]
        TFEngine = None,
        TFPylon=None
        return TFEngine, TFPylon
    
if __name__ == "__main__":
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

#    Generate a wing first to attach the engine to
    P = (0,0,0)
    SegmentNo = 10
    ChordFactor = 1
    ScaleFactor = 50
    Wing = liftingsurface.LiftingSurface(P, mySweepAngleFunctionAirliner, 
                                         myDihedralFunctionAirliner, 
                                         myTwistFunctionAirliner, 
                                         myChordFunctionAirliner, 
                                         myAirfoilFunctionAirliner, 
                                         SegmentNo=SegmentNo, ChordFactor=ChordFactor,
                                         ScaleFactor=ScaleFactor)#, TipRequired = True)

    display.DisplayShape(Wing.Shape, update=True)
    SpanStation = 0.3 # The engine is to be placed at 30% span
    EngineDia = 2.9
    NacelleLength = 1.95*EngineDia
    
    EngineSection, Chord = act.CutSect(Wing.Shape, SpanStation)
    print(type(EngineSection))
#    CEP = rs.CurveEndPoint(Chord)
    CEP = gp_Pnt(0, 0, 0)

    # Variables controlling the position of the engine with respect to the wing
    EngineCtrFwdOfLE = 0.98  
    EngineCtrBelowLE = 0.35
    Scarf_deg = 4
    Centreloc = [CEP.X()-EngineCtrFwdOfLE*NacelleLength,
            CEP.Y(), 
            CEP.Z()-EngineCtrBelowLE*NacelleLength]

#   Now build the engine and its pylon
    eng1 =  Engine(EngineSection, Chord,
                   CentreLocation=Centreloc,
                   ScarfAngle=Scarf_deg,
                   HighlightRadius=EngineDia/2.0,
                   MeanNacelleLength = NacelleLength)
    
    display.DisplayShape(act.make_edge(eng1._FC), update=True)
    display.DisplayShape(EngineSection, update=True, color='Black')    
    display.DisplayShape(eng1._FDisk, update=True)
    display.DisplayShape(eng1._TailCone)
    start_display()