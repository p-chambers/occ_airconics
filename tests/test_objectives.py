# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2017-08-31 16:41:11
# @Last Modified by:   p-chambers
# @Last Modified time: 2017-09-05 11:25:03
from airconics.objectives import *
from airconics import Topology
from SUAVE.Input_Output.OpenVSP import write
from SUAVE.Components.Wings import Wing
from SUAVE.Core import Units
import sys


def test_verticalwing_zerolift():
    json =   [{
    "primitive": "liftingsurface0",
    "args": {
      "X": 0.85,
      "Y": 0.0, 
      "Z": 0.3, 
      "ChordFactor": 0.9,
      "XScaleFactor": 0.115,
      "Rotation": 1.57, 
      "Type": "StraightWing"
    }
  }]
    topo = Topology()
    topo.from_json(json)
    vehicle = occ_to_suave(topo)
    import SUAVE
    results, angle_of_attacks, aerodynamics, state = suave_aero_analysis(vehicle)
    print(results.lift.total)
    assert(np.all(results.lift.total == 0))


def test_LD_ratio_independent_of_scale():
    """Tests that a wing """
    wing = Wing()

    def analyse_singlewing(ScaleFactor):
        c = 1. * ScaleFactor
        b = 2. * ScaleFactor

        wing.aspect_ratio            = b/c
        wing.thickness_to_chord      = 0.1
        wing.taper                   = 1.0
        wing.span_efficiency         = 0.9
        wing.spans.projected         = b * Units.meter
        wing.chords.root             = c * Units.meter
        wing.chords.tip              = c
        wing.chords.mean_aerodynamic = c/4. * Units.meter
        wing.areas.reference         = c * b * Units['meters**2']  
        wing.origin                  = [0.0,0.0,0.0] # meters
        wing.tag = "main_wing"

        suave_vehicle = SUAVE.Vehicle()

        suave_vehicle.append_component(wing)

        suave_vehicle.reference_area = wing.areas.reference

        if len(suave_vehicle.propulsors) == 0:
            # SUAVE bug fix: avoid zero division error in parasite drag calculation
            # if no engines exist (drag coeffs are normalised by this value, but are all zero anyway)
            empty_eng = SUAVE.Components.Energy.Networks.Turbofan()
            empty_eng.number_of_engines=0
            suave_vehicle.append_component(empty_eng)

        return suave_aero_analysis(suave_vehicle,
            analyses=SUAVE.Analyses.Aerodynamics.Fidelity_Zero)
        
    # Compute the L/D curve for two identical wings with different scale factors, and check
    # Whether the results are the same
    results, aoa, aerodynamics, state = analyse_singlewing(10)

    results2, aoa2, aerodynamics2, state2 = analyse_singlewing(50)

    LD1 = results.lift.total / results.drag.total
    LD2 = results2.lift.total / results2.drag.total

    assert(np.all(np.abs(LD1 - LD2) / LD1 < 0.01))


def test_symmetricwing_greater_lift():
    json1 =   [{
    "primitive": "liftingsurface0",
    "args": {
      "X": 0.0,
      "Y": 0.0, 
      "Z": 0.0, 
      "ChordFactor": 0.9,
      "XScaleFactor": 0.115,
      "Rotation": 0, 
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
      "XScaleFactor": 0.23,
      "Rotation": 0, 
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
    # check that the lift is greater than 75% larger
    print(vehicle.wings['main_wing'].spans.projected, vehicle.wings.main_wing.aspect_ratio, vehicle.reference_area, results.lift.total)
    print(vehicle2.wings['main_wing'].spans.projected, vehicle2.wings['main_wing'].aspect_ratio, vehicle2.reference_area, results2.lift.total)
    write(vehicle, 'vehicle1')
    write(vehicle2, 'vehicle2')
    # print(np.abs((results2.lift.total - results.lift.total)), results.lift.total, results2.lift.total)
    assert(np.all(np.abs((results2.lift.total - results.lift.total)/results.lift.total) > 0.75))
