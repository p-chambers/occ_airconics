# -*- coding: utf-8 -*-
"""
Classes for creating Airconics Engine objects

Created on Wed Feb 10 16:38:47 2016

@author: pchambers
"""
from . import primitives, AirCONICStools as act
from .liftingsurface import LiftingSurface
from .base import AirconicsShape
import numpy as np
from .examples import wing_example_transonic_airliner as wingex
from OCC.gp import gp_Pnt, gp_Vec, gp_OY, gp_Dir
from OCC.Geom import Handle_Geom_Circle
from OCC.GC import GC_MakeSegment
import logging

log = logging.getLogger(__name__)


class Engine(AirconicsShape):
    """A class for generating aircraft engine and pylon geometries.

    Currently only yields a turbofan engine with nacelle similar to that of an
    RR Trent 1000 / GEnx. Shapes produced include the nacelle, spinner cone,
    tail cone, Fan disk, Bypass disk, and pylon symmetry plane. The nacelle
    is produced by inclining an inlet disk by its scarf angle about the span-
    wise (y) axis and uniformly spacing airfoil 'ribs' before lofting a surface
    through them. The pylon is currently the symetry plane of a fully pylon
    only

    Parameters
    ----------
    HChord : OCC.Geom.Handle_Geom_TrimmedCurve
        The chord line at which the engine will be fitted. The result of
        OCC.GC.GC_MakeSegment.Value() (can be return from helper function
        CutSect from AirCONICStools).

    CentreLocation : list, length 3 (default=[0,0,0])
        Location of the centre of the inlet highlight disk

    ScarfAngle : scalar, deg (default=3)
        angle of inclination of engine intake (rotated around y axis)

    HighlightRadius : scalar (default=1.45)
        Intake highlight radius

    MeanNacelleLength : scalar (default=5.67)
        Mean length of the nacelle, to be used as the airfoil rib chordlength

    construct_geometry : bool
        If true, Build method will be called on construction

    SimplePylon : bool
        Simplifies the 787 style pylon for a single loft segment between 2
        airfoils

    PylonRotation : scalar (default 90)
        Optional dihedral angle of the pylon loft sections, in degrees 

    Attributes
    ----------
    _Components : dictionary of shapes

    Notes
    -----
    * Also calls the initialiser of parent class AirconicsShape which stores
      all keywords as attributes

    See also
    --------
    airconics.base.AirconicsShape, airconics.primitives.Airfoil
    """

    def __init__(self,
                 HChord=0,
                 CentreLocation=[0, 0, 0],
                 ScarfAngle=3,
                 HighlightRadius=1.45,
                 MeanNacelleLength=5.67,
                 construct_geometry=True,
                 SimplePylon=False,
                 PylonRotation=90
                 ):

        if HChord == 0:
            log.warning("No HChord specified to fit engine to: creating default")
            SP = gp_Pnt(MeanNacelleLength * 2.2, 0, HighlightRadius * 1.75)
            EP = gp_Pnt(MeanNacelleLength * 0.5, 0, HighlightRadius * 1.75)
            HChord = GC_MakeSegment(SP, EP).Value()

        # Add all kwargs as attributes
        super(Engine, self).__init__(components={},
                                     construct_geometry=construct_geometry,
                                     HChord=HChord,
                                     CentreLocation=CentreLocation,
                                     ScarfAngle=ScarfAngle,
                                     HighlightRadius=HighlightRadius,
                                     MeanNacelleLength=MeanNacelleLength,
                                     SimplePylon=SimplePylon,
                                     PylonRotation=PylonRotation
                                     )

    def Build(self):
        """Currently only calls BuildTurbofanNacelle.

        Notes
        -----
        May add options for other engine types
        """
        super(Engine, self).Build()
        self.BuildTurbofanNacelle()
        return None

    def BuildTurbofanNacelle(self):
        """
        The defaults yield a nacelle similar to that of an RR Trent 1000 / GEnx

        #TODO: break this down into modular function calls
        """
        CentreLocation = self.CentreLocation

        MeanNacelleLength = self.MeanNacelleLength
        HighlightRadius = self.HighlightRadius
        HighlightDepth = 0.12 * self.MeanNacelleLength
        SectionNo = 100

#         Draw the nacelle with centre of the intake highlight circle in 0,0,0
        HHighlight = act.make_circle3pt([0, 0, HighlightRadius],
                                        [0, -HighlightRadius, 0],
                                        [0, 0, -HighlightRadius])
        Highlight = HHighlight.GetObject()
        HHighlightCutterCircle = \
            act.make_circle3pt([0, 0, HighlightRadius * 1.5],
                               [0, -HighlightRadius * 1.5, 0],
                               [0, 0, -HighlightRadius * 1.5])
        HighlightCutterCircle = HHighlightCutterCircle.GetObject()
#         Fan disk for CFD boundary conditions
        FanCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength * 0.25, 0, 0)))
        wire = act.make_wire(act.make_edge(FanCircle))
        FanDisk = act.make_face(wire)
        self.AddComponent(FanDisk, 'FanDisk')

#         Aft outflow for CFD boundary conditions
        BypassCircle = Handle_Geom_Circle.DownCast(
            Highlight.Translated(gp_Vec(MeanNacelleLength * 0.85, 0, 0)))
        wire = act.make_wire(act.make_edge(BypassCircle))
        BypassDisk = act.make_face(wire)
        self.AddComponent(BypassDisk, 'BypassDisk')

#         Outflow cone
        TailConeBasePoint = np.array([MeanNacelleLength * 0.84, 0, 0])
        TailConeHeight = MeanNacelleLength * (1.35 - 0.84)
        TailConeRadius = HighlightRadius * 0.782
        TailCone = act.AddCone(TailConeBasePoint, TailConeRadius,
                               TailConeHeight)
        self.AddComponent(TailCone, 'TailCone')

#         Spinner cone
        SpinnerConeBasePoint = np.array([MeanNacelleLength * 0.26, 0, 0])
        SpinnerConeHeight = MeanNacelleLength * (0.26 - 0.08)
        SpinnerConeRadius = MeanNacelleLength * 0.09
        Spinner = act.AddCone(SpinnerConeBasePoint, SpinnerConeRadius,
                              SpinnerConeHeight, direction=gp_Dir(-1, 0, 0))
        self.AddComponent(Spinner, 'Spinner')
#
# Tilt the intake
        RotAx = gp_OY()
        Highlight.Rotate(RotAx, np.radians(self.ScarfAngle))
#        # Set up the disk for separating the intake lip later
        HighlightCutterCircle.Rotate(RotAx, np.radians(self.ScarfAngle))
        HighlightCutterDisk = act.PlanarSurf(HighlightCutterCircle)

        HighlightCutterDisk =\
            act.translate_topods_from_vector(HighlightCutterDisk,
                                             gp_Vec(HighlightDepth, 0, 0))

#         Build the actual airfoil sections to define the nacelle
        HighlightPointVector = act.Uniform_Points_on_Curve(
            Highlight.GetHandle(), SectionNo)

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

#        # Build the actual nacelle OML surface
#        EndCircle = act.points_to_bspline(TailPoints)    #Dont need this?
#        self._EndCircle = EndCircle
        Nacelle = act.AddSurfaceLoft(Sections)
        self.AddComponent(Nacelle, 'Nacelle')
#        TODO: Separate the lip
#        Cowling, HighlightSection = act.TrimShapebyPlane(Nacelle,
#                                                          HighlightCutterDisk,
#                                                          True)

#        Move the engine into its actual place on the wing
        self.TranslateComponents(gp_Vec(*CentreLocation))

        if self.SimplePylon:
            # Get the chord from its handle
            Chord = self.HChord.GetObject()
            CEP = Chord.EndPoint()
            # The End point where the airfoil is generated 
            CP1 = [MeanNacelleLength * 0.26 + CentreLocation[0],
                         CentreLocation[1],
                         CentreLocation[2] + HighlightRadius * 0.1]
            CP1_pt = gp_Pnt(*CP1)

            # vec = gp_Vec(Chord.StartPoint(), CP1_pt)
            # print(vec.Z())
            # print(vec.Y())
            # theta = np.degrees(np.arctan(vec.Z() / vec.Y()))

            Pylon_StartAf = primitives.Airfoil([CEP.X(), CEP.Y(), CEP.Z()], MeanNacelleLength * 1.35,
                                         self.PylonRotation, 0, Naca4Profile='0012',
                                         EnforceSharpTE=False)
            Pylon_EndAf = primitives.Airfoil(CP1, MeanNacelleLength * 1.35,
                                         self.PylonRotation, 0, Naca4Profile='0012',
                                         EnforceSharpTE=False)
            Pylon = act.AddSurfaceLoft([Pylon_StartAf, Pylon_EndAf])
            self.AddComponent(Pylon, 'Pylon')


        else:
            # Now build the pylon between the engine and the chord on the wing
            CP1 = gp_Pnt(MeanNacelleLength * 0.26 + CentreLocation[0],
                         CentreLocation[1],
                         CentreLocation[2] + HighlightRadius * 0.1)
            CP2 = gp_Pnt(MeanNacelleLength * 0.4 + CentreLocation[0],
                         CentreLocation[1],
                         HighlightRadius * 1.45 + CentreLocation[2])

            # Get the chord from its handle
            Chord = self.HChord.GetObject()
            CP3 = Chord.EndPoint()
            CP4 = Chord.StartPoint()
            self._pylonPts = [CP1, CP2, CP3, CP4]

           # Pylon wireframe
            tangents = np.array([[0, 0, 1], [1, 0, 0]])
            PylonTop = act.points_to_bspline([CP1, CP2, CP3, CP4],
                                             tangents=tangents)
            self._PylonTop = PylonTop
            PylonBase_LE = [CP1.X(), CP1.Y(), CP1.Z()]
            PylonAf = primitives.Airfoil(PylonBase_LE, MeanNacelleLength * 1.35,
                                         self.PylonRotation, 0, Naca4Profile='0012',
                                         EnforceSharpTE=False)
            self._PylonAf = PylonAf
            LowerTE = PylonAf.ChordLine.GetObject().EndPoint()
           # LowerTE = rs.CurveEndPoint(PylonChord)
            PylonTE = GC_MakeSegment(LowerTE, CP4).Value()
            self._PylonTE = PylonTE

            edges = [act.make_edge(PylonAf.ChordLine),
                     act.make_edge(PylonTop),
                     act.make_edge(PylonTE)]

            Pylon_symplane = act.make_face(act.make_wire(*edges))
            self.AddComponent(Pylon_symplane, 'Pylon_symplane')

          #   TODO: Pylon surface. Currently a flat plate at symmetry plane.
          #  This should be done with a plate surface (similar to NetworkSrf in
          #  Rhino), but I haven't got this to work yet
          #   Method 1: Sweep - gives the wrong shape
          #  Pylon_tip = GC_MakeCircle(gp_Ax1(CP4, gp_Dir(0, 1, 0)),
          #                            PylonAf.ChordLength*0.001
          #                            ).Value()
    
          #  Pylon_curve = PylonAf.Curve.GetObject()
          #  PylonAf_TE = act.make_edge(Pylon_curve.StartPoint(),
          #                             Pylon_curve.EndPoint())
          #  PylonAf_Face = act.PlanarSurf(PylonAf.Curve)
          #  PylonAf_closedwire = act.make_wire(act.make_edge(PylonAf.Curve),
          #                                     PylonAf_TE)
          #  sections = [PylonAf.Curve, PylonTE]
          #  spine = act.make_wire(act.make_edge(PylonTop))
          #  self.PylonLeft = act.make_pipe_shell(spine, sections)
    
          #   Move into place under the wing
          #  self._Translate(gp_Vec(0,-CentreLocation[1],0))
    
          # TODO: Mirror the pylon half surface
          #  PylonRight = act.mirror(PylonLeft, plane='xz')
          #  PylonAfCurve = act.AddTEtoOpenAirfoil(PylonAfCurve)
          #  PylonAfSrf = rs.AddPlanarSrf(PylonAfCurve)

        return None


if __name__ == "__main__":
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

#    Generate a wing first to attach the engine to
    P = (0, 0, 0)

    NSegments = 10
    ChordFactor = 1
    ScaleFactor = 44.56
    Wing = LiftingSurface(P, wingex.mySweepAngleFunctionAirliner,
                          wingex.myDihedralFunctionAirliner,
                          wingex.myTwistFunctionAirliner,
                          wingex.myChordFunctionAirliner,
                          wingex.myAirfoilFunctionAirliner,
                          NSegments=NSegments,
                          ChordFactor=ChordFactor,
                          ScaleFactor=ScaleFactor)

    Wing.Display(display)
    SpanStation = 0.3              # The engine is to be placed at 30% span
    EngineDia = 2.9
    NacelleLength = 1.95 * EngineDia

    EngineSection, HChord = act.CutSect(Wing['Surface'], SpanStation)
    Chord = HChord.GetObject()
    CEP = Chord.EndPoint()
    display.DisplayShape(Chord, update=True, color='black')
    # Variables controlling the position of the engine with respect to the wing
    EngineCtrFwdOfLE = 0.98
    EngineCtrBelowLE = 0.35
    Scarf_deg = 4
    Centreloc = [CEP.X() - EngineCtrFwdOfLE * NacelleLength,
                 CEP.Y(),
                 CEP.Z() - EngineCtrBelowLE * NacelleLength]

#   Now build the engine and its pylon
    eng1 = Engine(HChord,
                  CentreLocation=Centreloc,
                  ScarfAngle=Scarf_deg,
                  HighlightRadius=EngineDia / 2.0,
                  MeanNacelleLength=NacelleLength)

#    Display Engine Components:
    eng1.Display(display)

    start_display()
