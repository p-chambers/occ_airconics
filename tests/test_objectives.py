# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2017-08-31 16:41:11
# @Last Modified by:   p-chambers
# @Last Modified time: 2017-09-01 17:02:30
from airconics.objectives import *
from airconics import Topology

def test_verticalwing_zerolift():
    json =   [{
    "primitive": "liftingsurface0",
    "args": {
      "X": 0.85,
      "Y": 0.0, 
      "Z": 0.3, 
      "ChordFactor": 0.9,
      "XScaleFactor": 0.115,
      "Rotation": -1.57, 
      "Type": "StraightWing"
    }
  }]
    topo = Topology()
    topo.from_json(json)
    vehicle = occ_to_suave(topo)
    import SUAVE
    write(vehicle, 'vertical_wing')
    results, angle_of_attacks, aerodynamics, state = suave_aero_analysis(vehicle)
    print(results.lift.total)
    assert(np.all(results.lift.total == 0))


def test_symmetricwing_greater_lift():
    json1 =   [{
    "primitive": "liftingsurface0",
    "args": {
      "X": 0.85,
      "Y": 0.0, 
      "Z": 0.3, 
      "ChordFactor": 0.9,
      "XScaleFactor": 0.115,
      "Rotation": -1.57, 
      "Type": "StraightWing"
    }
  }]
    json2 =   [{"primitive": "mirror1", "args": {}
    },
    {"primitive": "liftingsurface0",
    "args": {
      "X": 0.0,
      "Y": 0.0, 
      "Z": 0.0, 
      "ChordFactor": 0.9,
      "XScaleFactor": 0.115,
      "Rotation": -1.57, 
      "Type": "StraightWing"
    }
  }]
    topo = Topology()
    topo.from_json(json1)
    vehicle = occ_to_suave(topo)
    results, angle_of_attacks, aerodynamics, state = suave_aero_analysis(vehicle)

    topo2 = Topology()
    topo2.from_json(json2)
    vehicle2 = occ_to_suave(topo2)
    results2, _, _, _ = suave_aero_analysis(vehicle2)
    # check that the lift is greater than 50% larger
    # assert(np.all(np.abs(results2.lift.total - results.lift.total)/results.lift.total > 0.5))
    raise NotImplementedError()
