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
from OCC.gp import gp_Pnt, gp_Vec, gp_OY
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
        Highlight = act.make_circle3pt([0,0,HighlightRadius],
                                  [0,-HighlightRadius,0],
                                  [0,0,-HighlightRadius]).GetObject()
        HighlightCutterCircle = act.make_circle3pt([0,0,HighlightRadius*1.5],
                                              [0,-HighlightRadius*1.5,0],
                                              [0,0,-HighlightRadius*1.5]).GetObject()
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
        SpinnerConeBasePoint = np.array([MeanNacelleLength*0.26, 0,0])
        SpinnerConeHeight    = MeanNacelleLength*(0.26-0.08)
        SpinnerConeRadius    = MeanNacelleLength*0.09
        Spinner = act.AddCone(SpinnerConeBasePoint,  SpinnerConeRadius, 
                              SpinnerConeHeight, direction=gp_Dir(-1, 0, 0))
        self._Spinner = Spinner

#        # Tilt the intake
        RotAx = gp_OY()
        Highlight.Rotate(RotAx, np.radians(self.ScarfAngle))
#        # Set up the disk for separating the intake lip later
        HighlightCutterCircle.Rotate(RotAx, np.radians(self.ScarfAngle))
        HighlightCutterDisk = act.PlanarSurf(HighlightCutterCircle)


        HighlightCutterDisk = translate_topods_from_vector(HighlightCutterDisk,
                                                           gp_Vec(HighlightDepth, 0,0))
#        
#        # Build the actual airfoil sections to define the nacelle
        HighlightPointVector = act.Uniform_Points_on_Curve(Highlight, SectionNo)
        
        Sections = []
        TailPoints = []
        Rotation = 0
        Twist = 0
        AirfoilSeligName = 'goe613'
        SmoothingPasses = 1
        
        Rotations = np.linspace(0, 360, SectionNo)
        
        for i, pt in enumerate(HighlightPointVector[0:2]):
            Chord = MeanNacelleLength - pt.X()
            print(pt.X(), pt.Y(), pt.Z(), Chord)
            Af = primitives.Airfoil([pt.X(), pt.Y(), pt.Z()], Chord,
                                     Rotations[i], Twist,
                                     SeligProfile=AirfoilSeligName)
            Sections.append(Af)
            TailPoints.append(Af.Curve.GetObject().EndPoint())
            
        self._sections = Sections
#        
#        # Build the actual nacelle OML surface
#        EndCircle = act.points_to_bspline(TailPoints)
#        self._EndCircle = EndCircle
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
#        # TODO: Clean-up
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
    display.DisplayShape(eng1._Spinner)
#    display.DisplayShape(eng1._EndCircle)
    for section in eng1._sections:
        display.DisplayShape(section.Curve, update=True, color='blue')
    
    
    start_display()