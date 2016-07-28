# -*- coding: utf-8 -*-
#
# Examples of how to use the airfoilfunct decorator from the liftingsurface
# module to obtain a function f_airfoil(epsilon), where epsilon is the
# leading edge-attached spanwise coordinate
#
# @Author: p-chambers
# @Date:   2016-07-28 15:08:10
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-07-28 16:29:42
from airconics.liftingsurface import airfoilfunct
from airconics.primitives import Airfoil


def simple_SweepFunct(eps):
    return 1


def simple_DihedralFunct(eps):
    return 1


def simple_ChordFunct(eps):
    return 1


def simple_TwistFunct(eps):
    return 1

# Below is the main point of this file: four different examples are used...

# Example 1: Generating a Uniform airfoil profile function


@airfoilfunct
def my_UniformSeligProfileFunction(eps):
    """Essentially, return all the profile specific arguments which define the
    airfoil profile at a given epsilon as a dictionary of KEYWORD: VALUE pairs

    The leading edge, twist, rotation, leading edge, and chord parameters do
    not need to be passed to this function, as this will be done by the
    lifting surface class.
    """
    return {'SeligProfile': 'b707a'}


# Example 2: Generating the CRM airfoil profile function
@airfoilfunct
def my_CRMProfileFunction(eps):
    return {'CRMProfile': True, 'CRM_Epsilon': eps}


# Example 3: Generating an airfoil function which returns linearly interpolated
# airfoils between two other sections
@airfoilfunct
def my_InterpAirfoilFunction(eps):
    af_root = Airfoil(Naca4Profile='0012')
    af_tip = Airfoil(Naca4Profile='0025')

    profile_dict = {'InterpProfile': True, 'Epsilon': eps, 'Af1': af_root,
                    'Af2': af_tip, 'Eps1': 0, 'Eps2': 1}
    return profile_dict


# Example 4: Generating an airfoil function which returns linearly interpolated
# airfoils between multiple other sections
@airfoilfunct
def my_InterpAirfoilFunction_multiple(eps):
    # I'll use three airfoil sections, located at eps=0, eps=0.5, eps=1.
    # Currently, this is safest using airfoils with equal numbers of points,
    # TODO: make this work for non-matching numbers of points (cubic interp?)
    af_1 = Airfoil(Naca4Profile='0012')
    af_2 = Airfoil(Naca4Profile='0025')
    af_3 = Airfoil(Naca4Profile='2408')

    if eps < 0.5:
        profile_dict = {'InterpProfile': True, 'Epsilon': eps, 'Af1': af_1,
                        'Af2': af_2, 'Eps1': 0, 'Eps2': 0.5}
    elif eps <= 1:
        profile_dict = {'InterpProfile': True, 'Epsilon': eps, 'Af1': af_2,
                        'Af2': af_3, 'Eps1': 0.5, 'Eps2': 1}

    return profile_dict


if __name__ == '__main__':
    from airconics import LiftingSurface
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()
    # Position the4  different wings at these locations:
    P1 = (0, 0, 0)
    P2 = (0, 0, 10)
    P3 = (0, 0, 20)
    P4 = (0, 0, 30)

    # Class definition
    NSeg = 10

    # Instantiate the class
    ScaleFactor = 50

    # The uniform airfoil wing (RED)
    Wing1 = LiftingSurface(P1, simple_SweepFunct,
                           simple_DihedralFunct,
                           simple_TwistFunct,
                           simple_ChordFunct,
                           my_UniformSeligProfileFunction,
                           SegmentNo=NSeg,
                           ScaleFactor=ScaleFactor)

    # The CRM airfoil wing (BLUE)
    Wing2 = LiftingSurface(P2, simple_SweepFunct,
                           simple_DihedralFunct,
                           simple_TwistFunct,
                           simple_ChordFunct,
                           my_CRMProfileFunction,
                           SegmentNo=NSeg,
                           ScaleFactor=ScaleFactor)

    # The interpolated airfoil wing (Yellow)
    Wing3 = LiftingSurface(P3, simple_SweepFunct,
                           simple_DihedralFunct,
                           simple_TwistFunct,
                           simple_ChordFunct,
                           my_InterpAirfoilFunction,
                           SegmentNo=NSeg,
                           ScaleFactor=ScaleFactor)

    # The multiple section interpolated wing (Aluminium)
    Wing4 = LiftingSurface(P4, simple_SweepFunct,
                           simple_DihedralFunct,
                           simple_TwistFunct,
                           simple_ChordFunct,
                           my_InterpAirfoilFunction_multiple,
                           SegmentNo=NSeg,
                           ScaleFactor=ScaleFactor)

    Wing1.Display(display, color='RED')
    Wing2.Display(display, color='BLUE')
    Wing3.Display(display, color='YELLOW')
    Wing4.Display(display, color=None)
    start_display()
