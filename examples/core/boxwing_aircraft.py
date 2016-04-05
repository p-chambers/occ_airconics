# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 16:50:52 2016

@author: pchambers
"""

# transonic_airliner.py ========================================================
# Parametric geometry covering some of the design space of 'tube and wing' type 
# transonic airliners. Run this as a script to generate geometries approximating 
# those of the B787 Dreamliner (-8 and -9) and Airbus A380 (uncomment the appro-
# priate line at the end of this file - see the header of the function for brief
# explanations of the design variables, if you wish to build your own instances
# of this parametric model.
#
# NOTE: For good results in terms of visualisation, select the 'Rendered' view
# in your Rhinoceros viewports. Better still, download the free Neon render 
# plugin, which should put a 'Raytraced with Neon' entry in your viewport type
# menu.
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2.1
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================
from OCC.Graphic3d import Graphic3d_NOM_ALUMINIUM
from OCC.gp import gp_Ax1, gp_Pnt, gp_Dir

from airconics.examples.wing_example_transonic_airliner import *
from airconics.examples.boxwing import *
from airconics import liftingsurface, engine, fuselage_oml
import airconics.AirCONICStools as act

import numpy as np


def transonic_airliner(display=None,
                       Propulsion=1,
                       EngineDia=2.9,
                       FuselageScaling=[55.902, 55.902, 55.902],
                       NoseLengthRatio=0.182,
                       TailLengthRatio=0.293,
                       WingScaleFactor=44.56,
                       WingChordFactor=1.0,
                       Topology=1,
                       SpanStation1=0.31,
                       SpanStation2=0.625,
                       EngineCtrBelowLE=0.3558,
                       EngineCtrFwdOfLE=0.9837,
                       Scarf_deg=3):
    """
    Parameters
    ----------
    Propulsion - int
        1 - twin
        2 - quad
    EngineDia - float
        Diameter of engine intake highlight
    FuselageScaling - list, length 3
        Fx, Fy, Fz scaling factors of fuselage
    NoseLengthRatio - scalar
        Proportion of forward tapering section of the fuselage 
    TailLengthRatio - scalar
        Proportion of aft tapering section of the fuselage
    WingScaleFactor - scalar
        Scale Factor of the main wing
    WingChordFactor - scalar
        Chord factor of the main wing
    Topology - int
        Topology = 2 should yield a box wing airliner - use with caution
    SpanStation - float
        Inboard engine at this span station
    SpanStation2 - float
        Outboard engine at this span station (ignored if Propulsion=1)
    EngineCtrBelowLE - float
        Engine below leading edge, normalised by the length of the nacelle -
        range: [0.35,0.5]
    EngineCtrFwdOfLE - float
        Engine forward of leading edge, normalised by the length of the nacelle
        range: [0.85,1.5]
    Scarf_deg - scalar # Engine scarf angle
    
    Returns
    -------
    """

    try:
        Fus = fuselage_oml.Fuselage(NoseLengthRatio, TailLengthRatio, 
                                    Scaling = FuselageScaling, 
                                    NoseCoordinates = [0,0,0], 
                                    CylindricalMidSection = True)
    except:
        print "Fuselage fitting failed - stopping."
        return None

    FuselageHeight = FuselageScaling[2]*0.105
    FuselageLength = FuselageScaling[0]
    FuselageWidth  = FuselageScaling[1]*0.106

    if Fus.OMLSurf is None:
        print "Failed to fit fuselage surface, stopping."
        return None

#    FSurf = rs.CopyObject(FuselageOMLSurf)
    
    # Position of the apex of the wing
    if FuselageHeight < 8.0:
        WingApex = [0.1748*FuselageLength,0,-0.0523*FuselageHeight] #787:[9.77,0,-0.307]
    else:
        WingApex = [0.1748*FuselageLength,0,-0.1*FuselageHeight] #787:[9.77,0,-0.307]


    # Set up the wing object, including the list of user-defined functions that
    # describe the spanwise variations of sweep, dihedral, etc.
    if Topology == 1:
        NSeg = 10
        Wing = liftingsurface.LiftingSurface(WingApex, mySweepAngleFunctionAirliner, 
                                            myDihedralFunctionAirliner, 
                                            myTwistFunctionAirliner, 
                                            myChordFunctionAirliner, 
                                            myAirfoilFunctionAirliner,
                                            SegmentNo=NSeg,
                                            ScaleFactor=WingScaleFactor,
                                            ChordFactor=WingChordFactor)
        RootChord = Wing.RootChord
    elif Topology == 2:
        raise NotImplementedError("Box Wing functionality is not yet ported to OCC_AirCONICS")
#        SegmentNo = 101
#        Wing = liftingsurface.LiftingSurface(WingApex, ta.mySweepAngleFunctionAirliner,
#        bw.myDihedralFunctionBoxWing, ta.myTwistFunctionAirliner,
#        ta.myChordFunctionAirliner, ta.myAirfoilFunctionAirliner, 
#        LooseSurf, SegmentNo, TipRequired = True)

    if Topology == 1:
        # Add wing to body fairing 
        WTBFXCentre = WingApex[0] + RootChord/2.0 + RootChord*0.1297 # 787: 23.8
        if FuselageHeight < 8.0:
            #Note: I made changes to this in OCC Airconics to get a better fit
            # - Paul
            WTBFZ = RootChord*0.009 #787: 0.2
            WTBFheight = 1.8*0.1212*RootChord #787:2.7
            WTBFwidth = 1.08*FuselageWidth
        else:

            WTBFZ = WingApex[2] + 0.005*RootChord
            WTBFheight = 0.09*RootChord 
            WTBFwidth = 1.15*FuselageWidth
    
        WTBFlength = 1.167*RootChord #787:26
        
#        print(WTBFXCentre, WTBFZ, WTBFlength, WTBFwidth, WTBFheight)
        WBF = act.make_ellipsoid([WTBFXCentre, 0, WTBFZ], WTBFlength, WTBFwidth, WTBFheight)
#        display.DisplayShape(gp_Pnt(WTBFXCentre, 0, WTBFZ), update=True)
#        display.DisplayShape(gp_Pnt(WTBFXStern, 0, WTBFZ), update=True)
#        display.DisplayShape(gp_Pnt(WTBFXCentre, WTBFwidth/2., WTBFZ), update=True)
#        display.DisplayShape(gp_Pnt(WTBFXCentre, 0, WTBFZ+WTBFheight/2.), update=True)

    
#        # Trim wing inboard section
        CutCirc = act.make_circle3pt([0,WTBFwidth/4.,-45], [0,WTBFwidth/4.,45], [90,WTBFwidth/4.,0])
        CutCircDisk = act.PlanarSurf(CutCirc)
        Wing.Shape = act.TrimShapebyPlane(Wing.Shape, CutCircDisk)
#        display.DisplayShape(cyl, update=True)
#    elif Topology == 2:
#        # Overlapping wing tips
#        CutCirc = rs.AddCircle3Pt((0,0,-45), (0,0,45), (90,0,0))
#        CutCircDisk = rs.AddPlanarSrf(CutCirc)
#        CutDisk = CutCircDisk[0]
#        rs.ReverseSurface(CutDisk,1)
#        rs.TrimBrep(WingSurf, CutDisk)
#
#
#
#
#    # TODO: Engine installation (nacelle and pylon)
    NacelleLength = 1.95*EngineDia
    if Propulsion == 1:
        SpanStations = [SpanStation1]
    elif Propulsion == 2:
        SpanStations = [SpanStation1, SpanStation2]
   
    engines = []
    for i, SpanStation in enumerate(SpanStations):
        EngineSection, Chord = act.CutSect(Wing.Shape, SpanStation)
        CEP = Chord.EndPoint()
        Centreloc = [CEP.X()-EngineCtrFwdOfLE*NacelleLength,
                    CEP.Y(), 
                    CEP.Z()-EngineCtrBelowLE*NacelleLength]
        eng =  engine.Engine(EngineSection, Chord,
               CentreLocation=Centreloc,
               ScarfAngle=Scarf_deg,
               HighlightRadius=EngineDia/2.0,
               MeanNacelleLength = NacelleLength)

        engines.append(eng)

#    # Script for generating and positioning the fin
#    # Position of the apex of the fin
    P = [0.6524*FuselageLength,0.003,FuselageHeight*0.384]
    #P = [36.47,0.003,2.254]55.902
    
    if Topology == 1:
        ScaleFactor = WingScaleFactor/2.032 #787:21.93
    elif Topology == 2:
        ScaleFactor = WingScaleFactor/3.5 
    
    SegmentNo = 100
    ChordFactor = 1.01#787:1.01
    Fin = liftingsurface.LiftingSurface(P, mySweepAngleFunctionFin,
                                        myDihedralFunctionFin,
                                        myTwistFunctionFin,
                                        myChordFunctionFin,
                                        myAirfoilFunctionFin,
                                        SegmentNo=SegmentNo,
                                        ChordFactor=ChordFactor,
                                        ScaleFactor=ScaleFactor)

#    Create the rotation axis centered at the apex point in the x direction
    RotAxis = gp_Ax1(gp_Pnt(*P), gp_Dir(1, 0, 0))
    
    Fin.Rotate(RotAxis, 90)

    if Topology == 1:
#        Tailplane
        P = [0.7692*FuselageLength,0.000,FuselageHeight*0.29]
        SegmentNo = 100
        ChordFactor = 1.01
        ScaleFactor = 0.388*WingScaleFactor #787:17.3
        TP = liftingsurface.LiftingSurface(P,
                                           mySweepAngleFunctionTP, 
                                           myDihedralFunctionTP,
                                           myTwistFunctionTP,
                                           myChordFunctionTP,
                                           myAirfoilFunctionTP,
                                           SegmentNo=SegmentNo,
                                           ChordFactor=ChordFactor,
                                           ScaleFactor=ScaleFactor)


#    # Mirror the geometry as required
    Wing2 = act.mirror(Wing.Shape, plane='xz', copy=True)
    try:
        # this try section allows box wing i.e. no tailplane
        TP2 = act.mirror(TP.Shape, plane='xz', copy=True)
    except:
        pass
    
    engines_left = []
    for eng in engines:
        engines_left.append(eng.MirrorComponents(plane='xz')) 
        
    
#    if Propulsion == 1:
#        print("No Engine Created yet")
#        for ObjId in EngineStbd:
#            act.MirrorObjectXZ(ObjId)
##        act.MirrorObjectXZ(PylonStbd)
#    elif Propulsion == 2:
#        raise NotImplementedError
#        for ObjId in EngineStbd1:
#            act.MirrorObjectXZ(ObjId)
#        act.MirrorObjectXZ(PylonStbd1)
#        for ObjId in EngineStbd2:
#            act.MirrorObjectXZ(ObjId)
#        act.MirrorObjectXZ(PylonStbd2)
#
        
#    display all entities:
    # Fuselage and wing-body fairing
    display.DisplayShape(Fus.OMLSurf, material=Graphic3d_NOM_ALUMINIUM)
    display.DisplayShape(WBF, material=Graphic3d_NOM_ALUMINIUM)

    #The Wings:
    display.DisplayShape(Wing.Shape, material=Graphic3d_NOM_ALUMINIUM)
    display.DisplayShape(Wing2, material=Graphic3d_NOM_ALUMINIUM)
    
    #The Tailplane:
    display.DisplayShape(TP.Shape, material=Graphic3d_NOM_ALUMINIUM)
    display.DisplayShape(TP2, material=Graphic3d_NOM_ALUMINIUM)
    
    #The Fin:
    display.DisplayShape(Fin.Shape, material=Graphic3d_NOM_ALUMINIUM)
    
    #The Engines: 
    for eng in engines + engines_left:
        eng.Display(display)
       
       
#    Build the return assembly (could change this later based on input 'tree':
    airliner = primitives.AirconicsShape()
    airliner.Components['Wing_right'] = Wing.Shape
    airliner.Components['Wing_left'] = Wing2
    airliner.Components['Fuselage'] = Fus.OMLSurf
    airliner.Components['WingBodyFairing'] = WBF
    airliner.Components['Fin'] = Fin.Shape
    airliner.Components['Tailplane_right'] = TP.Shape
    airliner.Components['Tailplane_left'] = TP2
    # Loop over the engines and write all components:
    for i, eng in enumerate(engines):
        for component in eng.Components:
            name = component + '_right' + str(i+1)
            airliner.Components[name] = eng.Components[component]
    for i, eng in enumerate(engines_left):
        for component in eng.Components:
            name = component + '_left' + str(i+1)
            airliner.Components[name] = eng.Components[component]    
    
    return airliner, engines_left


if __name__ == "__main__":
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
        
#    A few examples, instances of this parametric aircraft geometry:

#    '787-8'
    Airliner, engines_left = transonic_airliner(display)

#    '787-9'
#    transonic_airliner(FuselageScaling = [61.9, 55.902, 55.902])

#    'A380'
#     transonic_airliner(Propulsion = 2, 
#     FuselageScaling = [70.4, 67.36, 80.1], WingScaleFactor = 59.26)

#    This, for now, is just intended to stretch the model a bit in terms of 
#    topological variety - it is a box wing version of the 787-8. There is 
#    no serious design intent here, merely a hint of some of the possibilities
#    in this model.
#    transonic_airliner(WingScaleFactor = 66, WingChordFactor = 0.5, Topology =2)
    
    # Write step file:
#    for comp in Airliner.Components:
#        filename = '/home/pchambers/Documents/Vbox_share/Geometries/Step/' +\
#                    comp + '.stp'
#        act.export_STEPFile([Airliner.Components[comp]], filename)
#    shapes = Airliner.Components.values()
    act.export_STEPFile_Airconics([Airliner], '/home/pchambers/Documents/Vbox_share/Geometries/Airliner_airconicsexport.stp')
#    spinner = engines_left[0].Components['Spinner']
#    act.export_STEPFile(shapes, '/home/pchambers/Documents/Vbox_share/Geometries/Step/Airliner_exportstep.stp')
    
    # Step had issues, try stl:
#    Airliner.WriteComponents('/home/pchambers/Documents/Vbox_share/Airliner.stl')
    
#    from OCC.Graphic3d import (Graphic3d_EF_PDF,
#                           Graphic3d_EF_PostScript)
#    display.View.View().GetObject().Export('./Airliner.pdf', Graphic3d_EF_PDF)
    
    start_display()
    