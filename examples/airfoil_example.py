# -*- coding: utf-8 -*-
"""
Created on Tue Dec  8 12:56:10 2015

 Example script for generating an airfoil object given a set of coordinates spe-
 cified in a Selig format.
 ==============================================================================
 AirCONICS
 Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
 version 0.2.0
 Andras Sobester, 2015.
 Bug reports to a.sobester@soton.ac.uk or @ASobester please.
 ==============================================================================

@author: pchambers
"""
from airconics import primitives


# ==============================================================================
# Example: airfoil from Selig-formatted coordinate file. Leading edge point
# in origin, unit chord along x axis, no rotation around the x or y axes.
# Coordinates for Drela DAE11 low Reynolds number section, two smoothing
# iterations.
# ==============================================================================
LEPoint = [0., 0., 0.]
ChordLength = 1
Rotation = 0
Twist = 0
AirfoilSeligName = 'dae11'
# SmoothingPasses = 1   #TODO

# Instantiate class to set up a generic airfoil with these basic parameters
Af = primitives.Airfoil(LEPoint, ChordLength, Rotation, Twist,
                        SeligProfile=AirfoilSeligName)

# Visualisation with Python-OCC (ensure plot windows are set to qt4)
from OCC.Display.SimpleGui import init_display
display, start_display, add_menu, add_function_to_menu = init_display()

display.DisplayShape(Af.shape, update=True)
start_display()