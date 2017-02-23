# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2017-02-03 17:05:56
# @Last Modified by:   p-chambers
# @Last Modified time: 2017-02-20 15:52:58
# -*- coding: utf-8 -*-
# @Author: pc12g10
# @Date:   2016-08-11 14:19:53
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-10-06 15:51:08

import numpy as np
from airconics import liftingsurface
import airconics.AirCONICStools as act
from airconics.examples.tapered_wing import *
from OCC.BRepAlgoAPI import BRepAlgoAPI_Section
from OCC.gp import gp_Pnt
from OCC.TopAbs import TopAbs_EDGE
from OCC.TopExp import TopExp_Explorer
from OCC.TopoDS import topods_Edge
from OCC.Graphic3d import Graphic3d_NOM_ALUMINIUM, Graphic3d_NOM_SHINY_PLASTIC
from OCC.Display.SimpleGui import init_display

from multiprocessing import Pool, current_process
# ==============================================================================
# Essentially the AirCONICS jetstream wing with the end cut back and no loading
# (uniform dihedral), and a blended winglet added.
# ==============================================================================


def make_ribbed_wing(wing_oml, nribs):
    print("{} creating {} ribs".format(current_process(), nribs))

    # Post-fit a structure
    xmin, ymin, zmin, xmax, ymax, zmax = Wing.Extents()

    # y locations for the ribs
    y = np.linspace(ymin, ymax, nribs)[1:-1]

    ribs = []

    rib_thickness = 0.005   # Fraction of span
    dy = rib_thickness * (ymax - ymin)
    spine = act.make_wire(act.make_edge(gp_Pnt(0, 0, zmin), gp_Pnt(0, 0, zmax)))

    from OCC.BRepOffsetAPI import BRepOffsetAPI_ThruSections

    for j, y in enumerate(y):
        sections = []
        for iy in [y, y+dy]:
            wire = act.make_wire(act.make_edge(gp_Pnt(xmin, iy, zmax), gp_Pnt(xmax, iy, zmax)))
            cutplane = act.make_pipe_shell(spine, [wire])
            I = BRepAlgoAPI_Section(cutplane, wing_oml)
            I.ComputePCurveOn1(True)
            I.Approximation(True)
            I.Build()
            rib = I.Shape()
            edges = []
            exp = TopExp_Explorer(rib, TopAbs_EDGE)
            while(exp.More()):
                edges.append(topods_Edge(exp.Current()))
                exp.Next()
            sections.append(act.make_wire(edges))

        # create the rib
        generator = BRepOffsetAPI_ThruSections(True, False, 1e-6)
        generator.AddWire(sections[0])
        generator.AddWire(sections[1])
        generator.Build()
        rib = generator.Shape()
        ribs.append(rib)
    return ribs


if __name__ == "__main__":
    # Initialise the display
    display, start_display, add_menu, add_function_to_menu = init_display()
    import time

    apex = (0, 0, 0)

    # Class definition
    NSeg = 1

    # Instantiate the class
    ChordFactor = 0.6
    ScaleFactor = 7.951

    Wing = liftingsurface.LiftingSurface(apex, TaperedWingSweepFunction,
                                         TaperedWingDihedralFunction,
                                         TaperedWingTwistFunction,
                                         TaperedWingChordFunction,
                                         TaperedWingAirfoilFunction,
                                         SegmentNo=NSeg,
                                         ScaleFactor=ScaleFactor,
                                         ChordFactor=ChordFactor)

    Wing.Write('wing.stp')

    wing_oml = Wing['Surface']

    rib_ns = (10, 10, 10, 10, 10, 10, 10, 10)
    rib_lists = []

    import functools

    # Time the parallel computation
    t = time.time()
    # p = Pool()

    func = functools.partial(make_ribbed_wing, wing_oml)
    # create the wing + ribs
    res = map(func, rib_ns)
    # p.close()


    print("time taken to create 4 wings in parallel: {}".format(time.time() - t))

    rib_lists = res


    # ribs2 = []
    # # Do the serial calculation and check time
    # t = time.time()
    # for n in rib_ns:
    #     ribs2.append(make_ribbed_wing(wing_oml, n))
    # print("time taken to create 4 wings in series: {}".format(time.time() - t))



    display.DisplayShape(wing_oml, material=Graphic3d_NOM_SHINY_PLASTIC, color='white', transparency=0.9)

    from OCC.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.TopoDS import TopoDS_Compound, topods_Face, topods_Edge
    from OCC.TopAbs import TopAbs_FACE
    from OCC.TopLoc import TopLoc_Location
    from OCC.BRep import BRep_Builder, BRep_Tool
    from OCC.BRepBuilderAPI import BRepBuilderAPI_MakeEdge
    from OCC.StlAPI import StlAPI_Writer

    for i, rib in enumerate(rib_lists[-1]):
        # BRepMesh_IncrementalMesh(rib, 0.8)
        # builder = BRep_Builder()
        # comp = TopoDS_Compound()
        # builder.MakeCompound(comp)

        # bt = BRep_Tool()
        # ex = TopExp_Explorer(rib, TopAbs_FACE)
        # while ex.More():
        #     face = topods_Face(ex.Current())
        #     location = TopLoc_Location()
        #     facing = (bt.Triangulation(face, location)).GetObject()
        #     tab = facing.Nodes()
        #     tri = facing.Triangles()
        #     for i in range(1, facing.NbTriangles()+1):
        #         trian = tri.Value(i)
        #         index1, index2, index3 = trian.Get()
        #         for j in range(1, 4):
        #             if j == 1:
        #                 m = index1
        #                 n = index2
        #             elif j == 2:
        #                 n = index3
        #             elif j == 3:
        #                 m = index2
        #             me = BRepBuilderAPI_MakeEdge(tab.Value(m), tab.Value(n))
        #             if me.IsDone():
        #                 builder.Add(comp, me.Edge())
        #     ex.Next()
        display.DisplayShape(rib, Graphic3d_NOM_ALUMINIUM)

        # # write a stl file as well
        # stl_writer = StlAPI_Writer()
        # f = 'rib{}.stl'.format(i)
        # stl_ascii_format = False
        # stl_writer.Write(rib, f, stl_ascii_format)

    start_display()

