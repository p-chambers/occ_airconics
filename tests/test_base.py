# -*- coding: utf-8 -*-
"""
Uses a basic airconics shape object standard inputs as a test for base
class methods

Created on Fri Apr 29 10:04:41 2016

@author: pchambers
"""
import pytest
import airconics.AirCONICStools as act
from OCC.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeSphere
from OCC.gp import gp_Pnt
import os
from airconics.base import AirconicsCollection, AirconicsShape


@pytest.fixture
def create_AirconicsShape():
    """Populates a basic airconics shape with a unit cube striding by 1 in x y 
    and z from the origin, and a unit sphere centered at the origin"""
    cube = BRepPrimAPI_MakeBox(gp_Pnt(0,0,0), 1, 1, 1).Shape()
    sphere = BRepPrimAPI_MakeSphere(gp_Pnt(0,0,0), 1).Shape()
    shape = AirconicsShape(components={'cube': cube, 'sphere': sphere})
    return shape
    # ------------------------------------------------------


@pytest.fixture(params=[
    # list of file extension types
    ('stl', 'airconics_shape.stl'),
    ('stp', 'airconics_shape.stp')])
def filename_examples(request):
    return request.param

@pytest.fixture(params=[
    # a list of file extensions for airconics collection types
    ('stl', 'collection.stl'),
    ('stp', 'collection.stp')
    ])
def filename_collections(request):
    return request.param


def test_AirconicsShape_Write_SingleExport(create_AirconicsShape, filename_examples,
                                           tmpdir):
    shape = create_AirconicsShape
    ftype, filename = filename_examples
    p = tmpdir
    outfile = p.join(filename).strpath
    shape.Write(outfile, single_export=True)

    files = p.listdir()
    assert(len(files) == 1)
    assert(files[0] == outfile)

#     Could Add some content checks later to ensure written content is correct?



def test_AirconicsShape_Write_MultiExport(create_AirconicsShape, filename_examples,
                                           tmpdir):
    shape = create_AirconicsShape
    ftype, filename = filename_examples
    p = tmpdir
    outfile = p.join(filename).strpath
    shape.Write(outfile, single_export=False)

    files = p.listdir()
    assert(len(files) == 2)
    shapenames = shape.keys()
    
    base, ext = os.path.splitext(os.path.basename(outfile))
    outfiles_expect = [base + '_' + name + ext for name in shapenames]
    
    for subfile in files:
        f = os.path.basename(subfile.strpath)
        assert(f in outfiles_expect)


def test_AirconicsCollection_Write_SingleExport(create_AirconicsShape, filename_collections,
                                           tmpdir):
    shape = create_AirconicsShape
    # Note: this doesnt currently seem to output two distinct shapes?
    
    # Create a basic airconics collection with two shapes in
    collection = AirconicsCollection(parts={'shape1':shape, 'shape2':shape})
    ftype, filename = filename_collections
    p = tmpdir
    outfile = p.join(filename).strpath
    collection.Write(outfile, single_export=True)

    files = p.listdir()
    assert(len(files) == 1)
    assert(files[0] == outfile)

#     Could Add some content checks later to ensure written content is correct?


# Might leave this out as it only test airconicsshape write function anyway
#def test_AirconicsCollection_MultiExport(create_Airconics_Shape, filename_examples,
#                                           tmpdir):
#    shape = create_AirconicsShape
#    ftype, filename = filename_examples
#    p = tmpdir
#    outfile = p.join(filename).strpath
#    shape.Write(outfile, single_export=False)
#
#    files = p.listdir()
#    assert(len(files) == 6)
#    shapenames = shape.keys()
#    
#    base, ext = os.path.splitext(os.path.basename(outfile))
#    outfiles_expect = [base + '_' + name + ext for name in shapenames]
#    
#    for subfile in files:
#        f = os.path.basename(subfile.strpath)
#        assert(f in outfiles_expect)


