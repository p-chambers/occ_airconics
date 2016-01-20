# -*- coding: utf-8 -*-
"""
Created on Fri Jan 15 11:39:16 2016

Example functions for generating the lifting surfaces for the tail of a
transport aircraft (fin and tailplane external geometry). Also, this shows
that a specific planform can be reconstructed (the tail planform geometry
here is an approximation of the B787 tail geometry).

# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================

@author: pchambers
"""

import numpy as np
from airconics import primitives
from pkg_resources import resource_string


# Fin spaniwse definition functions
def myDihedralFunctionFin(Epsilon):
    return 0


def myTwistFunctionFin(Epsilon):
    return 0


def myChordFunctionFin(Epsilon):
    """User-defined function describing the variation of the fin chord as a
    function of the leading edge coordinate"""
    # Data for Chordlengths is included in the airconics distribution:
    #  load using pkg_resources
    rawdata = resource_string(__name__, 'airliner_fin_chords.dat')
    ChordLengths = np.fromstring(rawdata, sep='\n')

    EpsArray = np.linspace(0, 1, np.size(ChordLengths))
    return np.interp(Epsilon, EpsArray, ChordLengths)


def myAirfoilFunctionFin(Epsilon, LEPoint, ChordFunct, ChordFactor,
                         DihedralFunct, TwistFunct):
    """Defines the variation of cross section as a function of Epsilon"""

    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon) /
                          np.cos(np.radians(TwistFunct(Epsilon))))

    AirfoilSeligName = 'sc20010'

#    TODO : SmoothingPasses = 3
    Af = primitives.Airfoil(LEPoint, ChordLength=AirfoilChordLength,
                            Rotation=DihedralFunct(Epsilon),
                            Twist=TwistFunct(Epsilon),
                            SeligProfile=AirfoilSeligName)
    return Af


def mySweepAngleFunctionFin(Epsilon):
    """User-defined function describing the variation of the fin sweep angle as
    a function of the leading edge coordinate"""
    # Data for SweepAngles is included in the airconics distribution:
    #  load using pkg_resources
    rawdata = resource_string(__name__, 'airliner_fin_sweep.dat')
    SweepAngles = np.fromstring(rawdata, sep='\n')

    EpsArray = np.linspace(0, 1, np.size(SweepAngles))
    return np.interp(Epsilon, EpsArray, SweepAngles)


# Tailplane spaniwse definition functions

def myDihedralFunctionTP(Epsilon):
    return 7.6


def myTwistFunctionTP(Epsilon):
    return 0


def myChordFunctionTP(Epsilon):
    """User-defined function describing the variation of the tailplane chord as
    a function of the leading edge coordinate"""
    # Data for Chordlengths is included in the airconics distribution:
    #  load using pkg_resources
    rawdata = resource_string(__name__, 'airliner_TP_chords.dat')
    ChordLengths = np.fromstring(rawdata, sep='\n')

    EpsArray = np.linspace(0, 1, np.size(ChordLengths))
    return np.interp(Epsilon, EpsArray, ChordLengths)


def myAirfoilFunctionTP(Epsilon, LEPoint, ChordFunct, ChordFactor,
                        DihedralFunct, TwistFunct):
    """Defines the variation of cross section as a function of Epsilon"""

    AirfoilChordLength = (ChordFactor*ChordFunct(Epsilon) /
                          np.cos(np.radians(TwistFunct(Epsilon))))

    AirfoilSeligName = 'sc20010'

#    TODO : SmoothingPasses = 3
    Af = primitives.Airfoil(LEPoint, ChordLength=AirfoilChordLength,
                            Rotation=DihedralFunct(Epsilon),
                            Twist=TwistFunct(Epsilon),
                            SeligProfile=AirfoilSeligName)
    return Af


def mySweepAngleFunctionTP(Epsilon):
    """User-defined function describing the variation of the fin sweep angle as
    a function of the leading edge coordinate"""
    # Data for SweepAngles is included in the airconics distribution:
    #  load using pkg_resources
    rawdata = resource_string(__name__, 'airliner_TP_sweep.dat')
    SweepAngles = np.fromstring(rawdata, sep='\n')

    EpsArray = np.linspace(0, 1, np.size(SweepAngles))
    return np.interp(Epsilon, EpsArray, SweepAngles)
