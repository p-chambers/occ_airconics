# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-07-14 17:18:18
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-07-18 15:58:14
import pytest
from airconics.engine import Engine


# FIRST NEED TO GENERATE AN ENGINE TO USE IN ALL TESTS:
# ------------------------------------------------------
# Use the airconics wing chord values to fit the engine to (without generating
#  a wing):
@pytest.fixture
def empty_engine():
    eng = Engine(construct_geometry=False)
    return eng
    # ------------------------------------------------------

def test_BuildTurbofanNacelle(empty_engine):
    eng = empty_engine
    # TODO: Implement tests for turbofan function here
    eng.BuildTurbofanNacelle()

    # Check that all expected elements have been added (need to extend
    # to pylon components when they have been created)
    parts = ['FanDisk', 'BypassDisk', 'TailCone', 'Spinner', 'Nacelle']
    for part in parts:
        assert(part in eng), "part '{}' not found in engine".format(part)
