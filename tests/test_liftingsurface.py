# -*- coding: utf-8 -*-
"""
Created on Thu Jan 14 15:10:50 2016

@author: pchambers
"""
import numpy as np
import airconics.liftingsurface as ls
from airconics.examples.wing_example_transonic_airliner import *
import pytest


def test_GenerateLeadingEdge():
    # Test if GenerateLeadingEdge function matches previous version for
    # the Transonic Airliner example: Note this is only to 3dp
    LEPoints_ref = np.array([[0.000e+00,   0.000e+00,   0.000e+00],
                            [9.088e-02,   2.147e-03,   2.644e-04],
                            [1.752e-01,   3.580e-02,   4.508e-03],
                            [2.274e-01,   1.096e-01,   1.425e-02],
                            [2.795e-01,   1.834e-01,   2.463e-02],
                            [3.317e-01,   2.570e-01,   3.586e-02],
                            [3.838e-01,   3.304e-01,   4.815e-02],
                            [4.359e-01,   4.037e-01,   6.171e-02],
                            [4.881e-01,   4.766e-01,   7.675e-02],
                            [5.402e-01,   5.492e-01,   9.346e-02],
                            [5.924e-01,   6.213e-01,   1.121e-01],
                            [6.707e-01,   6.655e-01,   1.248e-01]])

    # Predefined Transonic Airliner example functions used here
    P = [0., 0., 0.]
    NSeg = 11
    Wing = ls.LiftingSurface(P, mySweepAngleFunctionAirliner,
                             myDihedralFunctionAirliner,
                             myTwistFunctionAirliner,
                             myChordFunctionAirliner,
                             myAirfoilFunctionAirliner, SegmentNo=NSeg)

    LEPoints = Wing.GenerateLeadingEdge()
    # Test First and final point first (easier to debug)
    assert(np.all(np.abs(LEPoints[1] - LEPoints_ref[1]) < 1e-3))
    assert(np.all(np.abs(LEPoints[-1] - LEPoints_ref[-1]) < 1e-3))

    # Now check that the entire array is less than 3 dp. out:
    assert(np.all(np.abs(LEPoints-LEPoints_ref) < 1e-3))


@pytest.mark.xfail
def test_liftingSurface_ChordScaleOptimizer():
    raise NotImplementedError