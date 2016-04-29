# -*- coding: utf-8 -*-
"""
Uses the multi-shape airconics engine class standard inputs as a test for base
class methods

Created on Fri Apr 29 10:04:41 2016

@author: pchambers
"""
import pytest
from airconics.engine import Engine
import airconics.AirCONICStools as act
from OCC.GC import GC_MakeSegment
from OCC.gp import gp_Pnt
import os
from airconics.base import AirconicsCollection

# FIRST NEED TO GENERATE AN ENGINE TO USE IN ALL TESTS:
# ------------------------------------------------------
# Use the airconics wing chord values to fit the engine to (without generating
#  a wing):


@pytest.fixture
def create_engine():
    Chord = GC_MakeSegment(gp_Pnt(21.24886, 7.96639, 1.07678),
                           gp_Pnt(12.32218, 7.96639, 1.07604)).Value().GetObject()

    CEP = Chord.EndPoint()
    # Variables controlling the position of the engine with respect to the wing
    EngineCtrFwdOfLE = 0.98
    EngineCtrBelowLE = 0.35
    Scarf_deg = 4
    EngineDia = 2.9
    NacelleLength = 1.95*EngineDia
    EngineCtrFwdOfLE = 0.9837
    EngineCtrBelowLE = 0.3558
    Centreloc = [CEP.X()-EngineCtrFwdOfLE*NacelleLength,
                 CEP.Y(),
                 CEP.Z()-EngineCtrBelowLE*NacelleLength]

    #   Now build the engine and its pylon
    eng1 = Engine(Chord,
                  CentreLocation=Centreloc,
                  ScarfAngle=Scarf_deg,
                  HighlightRadius=EngineDia/2.0,
                  MeanNacelleLength=NacelleLength)
    return eng1
    # ------------------------------------------------------


@pytest.fixture(params=[
    # list of file extension types
    ('stl', 'engine.stl'),
    ('stp', 'engine.stp')])
def filename_examples(request):
    return request.param

@pytest.fixture(params=[
    # a list of file extensions for airconics collection types
    ('stl', 'collection.stl'),
    ('stp', 'collection.stp')
    ])
def filename_collections(request):
    return request.param


def test_AirconicsShape_Write_SingleExport(create_engine, filename_examples,
                                           tmpdir):
    eng = create_engine
    ftype, filename = filename_examples
    p = tmpdir
    outfile = p.join(filename).strpath
    eng.Write(outfile, single_export=True)

    files = p.listdir()
    assert(len(files) == 1)
    assert(files[0] == outfile)

#     Could Add some content checks later to ensure written content is correct?



def test_AirconicsShape_Write_MultiExport(create_engine, filename_examples,
                                           tmpdir):
    eng = create_engine
    ftype, filename = filename_examples
    p = tmpdir
    outfile = p.join(filename).strpath
    eng.Write(outfile, single_export=False)

    files = p.listdir()
    assert(len(files) == 6)
    shapenames = eng.keys()
    
    base, ext = os.path.splitext(os.path.basename(outfile))
    outfiles_expect = [base + '_' + name + ext for name in shapenames]
    
    for subfile in files:
        f = os.path.basename(subfile.strpath)
        assert(f in outfiles_expect)


def test_AirconicsCollection_Write_SingleExport(create_engine, filename_collections,
                                           tmpdir):
    eng = create_engine
    # Note: this doesnt currently seem to output two distinct shapes?
    
    # Create a basic airconics collection with two engines in
    collection = AirconicsCollection(parts={'eng1':eng, 'eng2':eng})
    ftype, filename = filename_collections
    p = tmpdir
    outfile = p.join(filename).strpath
    collection.Write(outfile, single_export=True)

    files = p.listdir()
    assert(len(files) == 1)
    assert(files[0] == outfile)

#     Could Add some content checks later to ensure written content is correct?


# Might leave this out as it only test airconicsshape write function anyway
#def test_AirconicsCollection_MultiExport(create_engine, filename_examples,
#                                           tmpdir):
#    eng = create_engine
#    ftype, filename = filename_examples
#    p = tmpdir
#    outfile = p.join(filename).strpath
#    eng.Write(outfile, single_export=False)
#
#    files = p.listdir()
#    assert(len(files) == 6)
#    shapenames = eng.keys()
#    
#    base, ext = os.path.splitext(os.path.basename(outfile))
#    outfiles_expect = [base + '_' + name + ext for name in shapenames]
#    
#    for subfile in files:
#        f = os.path.basename(subfile.strpath)
#        assert(f in outfiles_expect)


