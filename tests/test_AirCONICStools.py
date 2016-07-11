# -*- coding: utf-8 -*-
"""
Created on Thu Jan  7 13:49:32 2016

@author: pchambers
"""
import airconics.AirCONICStools as act
import numpy as np


def test_coslin():
    abscissa = act.coslin(0.5, 8, 8)[0]
    ans = np.array([0.0,
                    0.01253604390908819,
                    0.04951556604879043,
                    0.1090842587659851,
                    0.1882550990706332,
                    0.2830581304412209,
                    0.3887395330218428,
                    0.49999999999999994,
                    0.5625,
                    0.625,
                    0.6875,
                    0.75,
                    0.8125,
                    0.875,
                    0.9375,
                    1.0])
    assert(np.all(np.abs(abscissa - ans) < 1e-10))


def test_Objects_Extents():
    box = act.BRepPrimAPI_MakeBox(1, 1, 1).Shape()
    X = np.array(act.ObjectsExtents(box))
    expected = np.array([0, 0, 0, 1, 1, 1])
    assert(np.all(np.abs(X - expected) < 1e-5))


