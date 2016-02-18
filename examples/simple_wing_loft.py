# -*- coding: utf-8 -*-
"""
Created on Tue Feb 16 13:41:19 2016

Simple Lofted Wing between airconics primitive classes using the AddSurfaceLoft
airconics tool

@author: pchambers
"""
if __name__ == "__main__":
    
    import airconics
    import airconics.AirCONICStools as act
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    
    # Add NACA 4 digit airfoils to loft between:
    Af2 = airconics.primitives.Airfoil([0.,0.,0.], ChordLength=3., Naca4Profile='2412')
    display.DisplayShape(Af2.Curve, update=True, color='GREEN');
    
    Af3 = airconics.primitives.Airfoil([0., 5., 0.], ChordLength=1., Naca4Profile='0012')
    display.DisplayShape(Af3.Curve, update=True, color='GREEN');
    
    Af4 = airconics.primitives.Airfoil([0., 6., 0.2], ChordLength=0.2, Naca4Profile='0012')
    display.DisplayShape(Af4.Curve, update=True, color='GREEN');
    
#    surf = act.AddSurfaceLoft([Af2, Af3, Af4])
    
    # Note that surf is a TOPO_DS Shape, and hence no surf.Shape is required for display
#    display.DisplayShape(surf, update=True);
    
    start_display()
