from airconics.topology import Topology_GPTools
import os
import airconics


if __name__ == '__main__':
    # Initialise the display
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    topo_tools = Topology_GPTools(MaxAttachments=4)

    # Note that we don't have to do anything special to turn a list of
    # dictionaries into json format before calling topo_tools.from_json:
    #  The primtives_array is already in json format!
    primitives_array = [
    {
        "primitive": "fuselage1",
        "args": {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0,
            "XScaleFactor": 1.0,
            "NoseLengthRatio": 0.182,
            "TailLengthRatio": 0.293,
            "FinenessRatio": 1.0
        }
    },
    {
        "primitive": "mirror3"
    },
    {
        "primitive": "engine0",
        "args": {
            "SpanStation": 0.8,
            "XChordFactor": 0.2,
            "DiameterLenRatio": 0.2,
            "PylonSweep": 0.2,
            "PylonLenRatio": 0.3,
            "Rotation": 0.5
        }
    },
    {
        "primitive": "liftingsurface0",
        "args": {
            "X": 0.27,
            "Y": 0.0,
            "Z": 0.68,
            "ScaleFactor": 0.42,
            "ChordFactor": 0.22,
            "Rotation": 0.05,
            "Type": "StraightWing"
        }
    },
    {
        "primitive": "liftingsurface1",
        "args": {
            "X": 0.77,
            "Y": 0.0,
            "Z": 0.5,
            "ChordFactor": 0.8,
            "ScaleFactor": 0.14,
            "Rotation": 0.0,
            "Type": "StraightWing"
        }
    },
    {
        "primitive": "fuselage3",
        "args": {
            "X": 1.0,
            "Y": 1.0,
            "Z": 1.0,
            "XScaleFactor": 2.8,
            "NoseLengthRatio": 0.182,
            "TailLengthRatio": 0.293,
            "FinenessRatio": 1.0
        }
    },
    {
        "primitive": "liftingsurface0",
        "args": {
            "X": 0.0,
            "Y": 0.0,
            "Z": 0.0,
            "ScaleFactor": 0.9,
            "ChordFactor": 0.31,
            "Rotation": 0.0,
            "Type": "TaperedWing"
        }
    },

    {
        "primitive": "liftingsurface0",
        "args": {
            "X": 0.85,
            "Y": 0.0,
            "Z": 0.9,
            "ChordFactor": 0.9,
            "ScaleFactor": 0.035,
            "Rotation": -1.0,
            "Type": "AirlinerFin"
        }
    },
    {
        "primitive": "liftingsurface0",
        "args": {
            "X": 0.85,
            "Y": 0.0,
            "Z": 0.1,
            "ChordFactor": 0.65,
            "ScaleFactor": 0.09,
            "Rotation": 1,
            "Type": "AirlinerFin"
    }
    }
    ]

    # Could use topo_tools.from_file instead:
    # fname = os.path.join(os.path.dirname(airconics.__file__),
    #                      'resources/configuration_app/presets/thunderbolt.json')

    topo = topo_tools.from_json(primitives_array)
    fus = topo['fuselage1_0']
    # chord = fus.get_spanstation_chord(0.8)
    from OCC.gp import gp_Vec, gp_Trsf, gp_Pnt, gp_Dir, gp_Pln
    from OCC.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCC.BRepAlgoAPI import BRepAlgoAPI_Section
    from OCC.TopAbs import TopAbs_EDGE
    from OCC.TopExp import TopExp_Explorer
    from OCC.TopoDS import topods_Edge
    from OCC.GC import GC_MakeSegment
    import airconics.AirCONICStools as act
    import numpy as np
    EngineSection, HChord = act.CutSect(fus['OML'], 0.8)

    HChord.GetObject().Translate(gp_Vec(0, 0.5, 0))
    trsf = gp_Trsf()
    trsf.SetTranslation(gp_Vec(0, -1, 0))

    # BRepBuilderAPI_Transform(EngineSection, trsf, False).Build()

    topo.Display(display)
    display.DisplayShape(HChord)
    display.DisplayShape(EngineSection)


    display.FitAll()
    start_display()
