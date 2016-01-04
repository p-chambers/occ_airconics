# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 15:52:58 2015

 LIFTINGSURFACE.PY ============================================================
 This module contains the definition of the class of 3d lifting surfaces.
 This class can be instantiated to generate wings, tailplanes, fins, propeller-
 or rotor blades, etc.

 This is an OCC_AirCONICS file, based on the Rhino 'AirCONICS' plugin
 by A. Sobester: https://github.com/sobester/AirCONICS
 ==============================================================================

@author: pchambers
"""
import primitives
import AirCONICStools as act


class LiftingSurface:

    def __init__(self, ApexPoint,
                 SweepFunct,
                 DihedralFunct,
                 TwistFunct,
                 ChordFunct,
                 AirfoilFunct,
                 LooseSurf=1,
                 SegmentNo=11,
                 TipRequired = True):

        self.ApexPoint = ApexPoint
        self.SweepFunct = SweepFunct
        self.DihedralFunct = DihedralFunct
        self.TwistFunct = TwistFunct
        self.ChordFunct = ChordFunct
        self.AirfoilFunct = AirfoilFunct
        self.LooseSurf = LooseSurf
        self.SegmentNo = SegmentNo
        self.TipRequired = TipRequired

#        self._CreateConstructionGeometry()
        self.GenerateLiftingSurface(ChordFactor, ScaleFactor OptimizeChordScale)
        
        def GenerateLiftingSurface(self, ChordFactor, ScaleFactor,
                                   OptimizeChordScale=0):
            """
            """
        
        