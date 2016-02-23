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
from OCC.gp import gp_Pnt, gp_Vec, gp_OY, gp_Dir, gp_Ax1
from OCC.Geom import Handle_Geom_Circle



# Will add a base class for rotating engine etc. later for inheritance
class Engine(primitives.AirconicsShape):
    def __init__(self,
                 EngineSection,
                 Chord, 
                 CentreLocation=[0,0,0],
                 ScarfAngle=3,
                 HighlightRadius=1.45,
                 MeanNacelleLength=5.67):
        """Constructor for the engine class"""
        
        #Inherit Base class variables
        super(Engine, self).__init__()
        
        self.EngineSection = EngineSection
        self.Chord = Chord
        self.CentreLocation = CentreLocation
        self.ScarfAngle = ScarfAngle
        self.HighlightRadius = HighlightRadius
        self.MeanNacelleLength = MeanNacelleLength
        self.Build_Engine()
        
    def Build_Engine(self):
        self.BuildTurbofanNacelle()
        return None
    
    def BuildTurbofanNacelle(self):
        """The defaults yield a nacelle similar to that of an RR Trent 1000 / GEnx
        """
        CentreLocation = self.CentreLocation

        MeanNacelleLength = self.MeanNacelleLength
        HighlightRadius = self.HighlightRadius
        HighlightDepth = 0.12 * self.MeanNacelleLength
        SectionNo = 100
        
#         Draw the nacelle with the centre of the intake highlight circle in 0,0,0
        Highlight = act.make_circle3pt([0,0,HighlightRadius],
                                  [0,-HighlightRadius,0],
                                  [0,0,-HighlightRadius]).GetObject()
        HighlightCutterCircle = act.make_circle3pt([0,0,HighlightRadius*1.5],
                                              [0,-HighlightRadius*1.5,0],
                                              [0,0,-HighlightRadius*1.5]).GetObject()
#         Fan disk for CFD boundary conditions
        FanCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength*0.25, 0, 0)))
        wire = act.make_wire(act.make_edge(FanCircle))
        FanDisk = act.make_face(wire)
        self.Components['FanDisk'] = FanDisk
        
#         Aft outflow for CFD boundary conditions
        BypassCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength*0.85, 0, 0)))
        wire = act.make_wire(act.make_edge(BypassCircle))
        BypassDisk = act.make_face(wire)#        rs.DeleteObjects([FanCircle, BypassCircle])
        self.Components['BypassDisk'] = BypassDisk
#    
#         Outflow cone
        TailConeBasePoint = np.array([MeanNacelleLength*0.84, 0,0])
        TailConeHeight    = MeanNacelleLength*(1.35-0.84)
        TailConeRadius    =  HighlightRadius*0.782
        TailCone = act.AddCone(TailConeBasePoint, TailConeRadius, TailConeHeight)
        self.Components['TailCone'] = TailCone
#         Spinner cone
        SpinnerConeBasePoint = np.array([MeanNacelleLength*0.26, 0,0])
        SpinnerConeHeight    = MeanNacelleLength*(0.26-0.08)
        SpinnerConeRadius    = MeanNacelleLength*0.09
        Spinner = act.AddCone(SpinnerConeBasePoint,  SpinnerConeRadius, 
                              SpinnerConeHeight, direction=gp_Dir(-1, 0, 0))
        self.Components['Spinner'] = Spinner

#         Tilt the intake
        RotAx = gp_OY()
        Highlight.Rotate(RotAx, np.radians(self.ScarfAngle))
#        # Set up the disk for separating the intake lip later
        HighlightCutterCircle.Rotate(RotAx, np.radians(self.ScarfAngle))
        HighlightCutterDisk = act.PlanarSurf(HighlightCutterCircle)


        HighlightCutterDisk = act.translate_topods_from_vector(HighlightCutterDisk,
                                                           gp_Vec(HighlightDepth, 0,0))
#        
#         Build the actual airfoil sections to define the nacelle
        HighlightPointVector = act.Uniform_Points_on_Curve(Highlight.GetHandle(), SectionNo)
        
        Sections = []
        TailPoints = []
        Twist = 0
        AirfoilSeligName = 'goe613'
        
        Rotations = np.linspace(0, 360, SectionNo)
        
        for i, pt in enumerate(HighlightPointVector):
            AfChord = MeanNacelleLength - pt.X()
            Af = primitives.Airfoil([pt.X(), pt.Y(), pt.Z()], AfChord,
                                     Rotations[i], Twist,
                                     SeligProfile=AirfoilSeligName).Curve
            Sections.append(Af)
            TailPoints.append(Af.GetObject().EndPoint())
            
        self._sections = Sections
#        
#        # Build the actual nacelle OML surface
#        EndCircle = act.points_to_bspline(TailPoints)    #Dont need this?
#        self._EndCircle = EndCircle
        Nacelle =  act.AddSurfaceLoft(Sections)
        self.Components['Nacelle'] = Nacelle
#        TODO: Separate the lip
#        Cowling, HighlightSection = act.TrimShapebyPlane(Nacelle, HighlightCutterDisk, True)

#        Move the engine into its actual place on the wing (Fuse first?)
        self.TranslateComponents(gp_Vec(*CentreLocation))         
#        
#        Now build the pylon between the engine and the specified chord on the wing
        CP1 = gp_Pnt(MeanNacelleLength*0.26+CentreLocation[0],
                     CentreLocation[1], CentreLocation[2]+HighlightRadius*0.1)
        CP2 = gp_Pnt(MeanNacelleLength*0.4+CentreLocation[0], CentreLocation[1],
               HighlightRadius*1.45+CentreLocation[2])
        CP3 = self.Chord.EndPoint()
        CP4 = self.Chord.StartPoint()
#        CP4 = gp_Pnt((CP3.X()+CP5.X())/2., 
#                     (CP3.Y()+CP5.Y())/2.,
#                     (CP3.Z()+CP5.Z())/2.)# Adding a mid point to make the curve behave better
        self._pylonPts = [CP1, CP2, CP3, CP4]
#    

        
 
#        Pylon wireframe
        tangents = np.array([[0,0,1],[1, 0, 0]])
        PylonTop = act.points_to_bspline([CP1, CP2, CP3, CP4], tangents=tangents)
        self._PylonTop = PylonTop
        PylonBase_LE = [CP1.X(), CP1.Y(), CP1.Z()]
        PylonAf = primitives.Airfoil(PylonBase_LE,MeanNacelleLength*1.35, 90,
                                     0, Naca4Profile='0012', EnforceSharpTE=False)
        self._PylonAf = PylonAf
        LowerTE = PylonAf.ChordLine.GetObject().EndPoint()
#        LowerTE = rs.CurveEndPoint(PylonChord)
        from OCC.GC import GC_MakeSegment
        PylonTE = GC_MakeSegment(LowerTE, CP4).Value()
        self._PylonTE = PylonTE
#        # Create the actual pylon surface       
        # Method 1: Sweep - gives the wrong shape
#        Pylon_tip = GC_MakeCircle(gp_Ax1(CP5, gp_Dir(0,1,0)), PylonAf.ChordLength*0.001).Value()

        Pylon_curve = PylonAf.Curve.GetObject()
        PylonAf_TE = act.make_edge(Pylon_curve.StartPoint(), Pylon_curve.EndPoint())
#        PylonAf_Face = act.PlanarSurf(PylonAf.Curve)
        
        
#        PylonAf_closedwire = act.make_wire(act.make_edge(PylonAf.Curve), PylonAf_TE)
#         Add a centre plane and trim the curve
        edges = [act.make_edge(PylonAf.ChordLine),
                 act.make_edge(PylonTop),
                 act.make_edge(PylonTE)]
        trimplane = act.make_face(act.make_wire(*edges))
#        wire = act.TrimShapebyPlane(PylonAf_closedwire,trimplane, pnt=gp_Pnt(0, -10, 0))
        self.Components['Pylon_Flatplate'] = trimplane
#        self._temp_halfsection = wire
        
#        sections = [PylonAf.Curve, PylonTE]
#        spine = act.make_wire(act.make_edge(PylonTop))
##        support = act.make_wire(act.make_edge(PylonTE))
##        self.PylonLeft = act.make_pipe_shell(spine, sections)
#        from OCC.BRepOffsetAPI import BRepOffsetAPI_MakePipeShell
#     
#        builder = BRepOffsetAPI_MakePipeShell(spine)
#        builder.C
#        builder.Add(PylonAf_closedwire)
#  
##        poly = BRepBuilderAPI_MakePolygon(p1, p2, p3).Wire()
#        
#        
#        builder.Add(act.make_vertex(CP5))
#        builder.Build()
#        self.PylonLeft = builder.Shape()
#        Tip circle:
        
#        self.PylonLeft = act.Add_Network_Surface([wire] + edges)
#        sections = 
#        spine = act.make_wire(act.make_edge(PylonTop))
#        support = act.make_wire(act.make_edge(PylonTE))
        
#        Method 3: Evolved
#        from OCC.BRepOffsetAPI import BRepOffsetAPI_MakeEvolved
#        from OCC.GeomAbs import GeomAbs_C0, GeomAbs_G2
#
#        print(type(wire))
#        continuity = GeomAbs_G2
#        filler = BRepOffsetAPI_MakeEvolved(act.make_wire(act.make_edge(PylonTop)), sections)
##        filler.Add(trimplane, GeomAbs_G2)
#        
##        filler.Add(e, continuity)
##        for edge in edges:
##            filler.Add(edge, continuity)
#        filler.Build()
#        face = filler.Shape()
#        self.PylonLeft = face
            

        # Move into place under the wing
#        self._Translate(gp_Vec(0,-CentreLocation[1],0))
        
        
#        PylonRight = act.mirror(PylonLeft, plane='xz')
#        PylonAfCurve = act.AddTEtoOpenAirfoil(PylonAfCurve)
#        PylonAfSrf = rs.AddPlanarSrf(PylonAfCurve)
#
#        # TODO: Clean-up
#        
#        TFEngine = [Cowling, HighlightSection, TailCone, FanDisk, Spinner, BypassDisk]
#        TFPylon = [PylonLeft, PylonRight, PylonAfSrf]

        
        TFEngine = None,
        TFPylon=None
        return TFEngine, TFPylon
        

    
#    def Display(self, context, material=Graphic3d_NOM_ALUMINIUM):
#        """Displays all components of the engine to input context
#        Parameters
#        ----------
#        meterial - OCC.Graphic3d_NOM_* type
#            The material for display: note some renderers do not allow this"""
#        for component in self.Components:
#            ais = AIS_Shape(self.Components[component])
#            ais.SetMaterial(material)
#            try:
#                context.Context.Display(ais.GetHandle())
#            except:
#                context.DisplayShape(self.Components[component])
#    
    
if __name__ == "__main__":
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

#    Generate a wing first to attach the engine to
    P = (0,0,0)

    SegmentNo = 10
    ChordFactor = 1
    ScaleFactor = 44.56
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
    eng1 =  Engine(EngineSection, Chord,
                   CentreLocation=Centreloc,
                   ScarfAngle=Scarf_deg,
                   HighlightRadius=EngineDia/2.0,
                   MeanNacelleLength = NacelleLength)
    
#    Display Engine Components:
    eng1.Display(display)
    
#    Display pylon components:
    display.DisplayShape(eng1._PylonAf.Curve)
    display.DisplayShape(eng1._PylonTop)
    display.DisplayShape(eng1._PylonTE)  
    display.DisplayShape(EngineSection, color='black')

#    display.DisplayShape(eng1._Pylon_Flatplate)

#    eng1.Display(display)

    start_display()