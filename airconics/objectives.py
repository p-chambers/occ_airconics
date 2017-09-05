# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2017-08-31 11:34:16
# @Last Modified by:   p-chambers
# @Last Modified time: 2017-09-04 18:56:03
import SUAVE
from SUAVE.Attributes.Gases.Air import Air
import numpy as np
import matplotlib.pyplot as plt
from SUAVE.Core import Data, Units
import logging

# logger = logging.getLogger(__name__)

try:
    import vsp_g as vsp
except ImportError:
    # VSP not available on path is not a catastrophic, so allow airconics to
    # build without it
    logger.warning("VSP modules not found in path. ")
    pass


def occ_to_suave(topo):
    """Uses the occ topo.ToSuave method, but includes some fail-safes for
    steering the objective function.

    A penalty is returned should the vehicle not be suitable for analysis
    (i.e., it should be skipped)"""
#     try:
    suave_vehicle = topo.ToSuave()
    if len(suave_vehicle.propulsors) == 0:
        # SUAVE bug fix: avoid zero division error in parasite drag calculation
        # if no engines exist (drag coeffs are normalised by this value, but
        # are all zero anyway)
        empty_eng = SUAVE.Components.Energy.Networks.Turbofan()
        empty_eng.number_of_engines = 0

        suave_vehicle.append_component(empty_eng)
    # Obtain the "main wing" (used for aero analysis) as the one with maximum
    # projected area:
    wings = {k: v for k, v in topo.iteritems() if hasattr(
        v, "LSP_area") and not v.MirrorComponentsXZ}
    main_wing_key = max(wings.iteritems(), key=lambda kv: kv[1].LSP_area)[0]
    # Remove the key:value pair attributed to the previous key, and add a new
    # pair with the
    # name "main_wing"
    suave_vehicle.wings['main_wing'] = suave_vehicle.wings.pop(main_wing_key)
    # SUAVE 1.0 release VLM requires the tag to be also set as main wing:
    suave_vehicle.wings['main_wing'].tag = 'main_wing'

    if len(suave_vehicle.fuselages) > 0:
        # Rename the fuselage: NOTE THAT THIS MAY AFFECT THE OUT ANALYSIS, SINCE
        # SUAVE ONLY SEEMS TO ACCOUNT FOR A SINGLE FUSELAGE WITH THE TAG "FUSELAGE"
        main_fuselage_key = max(suave_vehicle.fuselages.iteritems(), key=lambda kv: kv[1].areas.wetted)[0]
        suave_vehicle.fuselages['fuselage'] = suave_vehicle.fuselages.pop(main_fuselage_key)
        suave_vehicle.fuselages['fuselage'].tag = 'fuselage'
    return suave_vehicle


def suave_aero_analysis(suave_vehicle,
                   test_num=11,
                   alpha_min=-5,
                   alpha_max=10,
                   analyses=SUAVE.Analyses.Aerodynamics.Fidelity_Zero,
                   Mc=0.85,
                   rho=0.003996,
                   mu=1.601e-5,
                   T=297.8,
                   pressure=0.0287e4):
    """Calculates aerodynamic drag and lift angles of attack between

    Underlying SUAVE analysis is, in fact, based on a surrogate model, and as
    such, computation time used to perform a small number of test points will
    be dominated by initial analysis and surrogate generation.

    Default parameters used in analyses are the flight conditions of the
    Boeing 787, i.e., Mach 0.85, altitude 40,000ft (standard ISA conditions)

    Parameters
    ----------

    test_num : int
        The number of points to obtain Cl and Cd
    alpha_min, alpha_max : int
        The angle of attack range
    analyses : Suave.Analyses.Aerodynamics.* [default Fidelity_Zero]
        The fidelity of analysis to use from SUAVE
    Mc : Scalar
        Cruise Mach number
    rho : scalar
        density
    mu : scalar
        air dynamic viscosity
    """
    aerodynamics = analyses()
    aerodynamics.geometry = suave_vehicle

    # This should probably be calculated
    aerodynamics.settings.oswald_efficiency_factor = 0.8

    aerodynamics.initialize()

    # Reshape the angle of attack array to a N by 1 column vector (suave
    # seems to recommend this for whatever reason):
    angle_of_attacks = np.reshape(np.radians(np.linspace(-5, 10, test_num)),
        (test_num, 1))

    # Cruise conditions (except Mach number)
    state = SUAVE.Analyses.Mission.Segments.Conditions.State()
    state.conditions = SUAVE.Analyses.Mission.Segments.Conditions.Aerodynamics()

    state.expand_rows(test_num)

    # --------------------------------------------------------------------
    # Initialize variables needed for CL and CD calculations
    # Use a seeded random order for values
    # --------------------------------------------------------------------

    # Cruise condition of 787:
    Mcs = np.ones_like(angle_of_attacks) * Mc
    rhos = np.ones_like(angle_of_attacks) * rho
    mus = np.ones_like(angle_of_attacks) * mu
    Ts = np.ones_like(angle_of_attacks) * T
    pressures = np.ones_like(angle_of_attacks) * pressure

    air = Air()
    a = air.compute_speed_of_sound(T, pressure)

    re = rhos * a * Mcs / mus
#         print("Speed of sound = {}. Re = {}".format(a, re))

    state.conditions.freestream.mach_number = Mcs
    state.conditions.freestream.density = rhos
    state.conditions.freestream.dynamic_viscosity = mus
    state.conditions.freestream.temperature = Ts
    state.conditions.freestream.pressure = pressures
    state.conditions.freestream.reynolds_number = re

    state.conditions.aerodynamics.angle_of_attack = angle_of_attacks

    # --------------------------------------------------------------------
    # Surrogate
    # --------------------------------------------------------------------
    # call the aero model
    results = aerodynamics.evaluate(state)

    return results, angle_of_attacks, aerodynamics, state


def plot_CL_CD(topo, analyses=SUAVE.Analyses.Aerodynamics.Fidelity_Zero):
    """Calculates and plots the CL/CD curves for a range of angles
    """
    suave_vehicle = occ_to_suave(topo)
    results, aoa, aerodynamics, state = suave_aero_analysis(suave_vehicle, analyses=analyses)

    # build a polar for the markup aero
    polar = Data()
    CL = results.lift.total
    CD = results.drag.total
    polar.lift = CL
    polar.drag = CD

    aoa = np.degrees(np.reshape(aoa, len(aoa)))
    # CL = CL[1:-1]
    CL = np.reshape(CL, len(CL))
    CD = np.reshape(CD, len(CD))

    factors = np.polyfit(aoa, CL, 1)
    p = np.poly1d(factors)

    aoa_sample = np.linspace(min(aoa), max(aoa), 11)
    cl_sample = p(aoa_sample)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    l1 = ax.plot(aoa, CL, 'kx', label='C_L')
    l2 = ax.plot(aoa_sample, cl_sample, 'k-', label='C_L Fit')
    ax.set_xlabel(r"$\alpha$ (deg)")
    ax.set_ylabel(r"$C_L$")

    ax2 = ax.twinx()
    l3 = ax2.plot(aoa, CD, 'ko', label='C_D')
    ax2.set_ylabel(r'$C_D$')
    lns = l1 + l2 + l3
    labs = [l.get_label() for l in lns]
    ax2.legend(lns, labs, loc=9)

    fig3 = plt.figure()
    axes = fig3.add_subplot(111)
    axes.plot(CD, CL, 'bo-')  # ,CD_old,CL_old,'*')
    axes.set_xlabel('$C_D$')
    axes.set_ylabel('$C_L$')
    # axes.set_ylim(0, max(CL))
    return CL, CD


def LD_ratio(topo):
    """
    """
    try:
        suave_vehicle = occ_to_suave(topo)
        results, aoa, aerodynamics, state = suave_aero_analysis(suave_vehicle,
            analyses=SUAVE.Analyses.Aerodynamics.Fidelity_Zero)
        return (results.lift.total / results.drag.total)[
            np.abs(aoa - np.ones_like(aoa) * 3.0).argmin()]

    except (ValueError, TypeError):
        # These geometries either failed construction or analysis: pay a
        # penalty (this may incorrectly steer it away from otherwise optimal
        # solutions, so will need to fix this later)
        return -1000


def write(vehicle, tag):
    """
    Copied directly from SUAVE. For some reason, the version of this function
    in the main repository is not the same as the version in release 1.0.0,
    (the latter does not work)

    This writes a SUAVE vehicle to OpenVSP format. It will take wing segments
    into account
    if they are specified in the vehicle setup file.

    Assumptions:
    Vehicle is composed of conventional shape fuselages, wings, and propulsors.
    Any propulsor
    that should be created in tagged as 'turbofan'.
    Source:
    N/A
    Inputs:
    wings.*.    (* is all keys)
      origin                                  [m] in all three dimensions
      spans.projected                         [m]
      chords.root                             [m]
      chords.tip                              [m]
      sweeps.quarter_chord                    [radians]
      twists.root                             [radians]
      twists.tip                              [radians]
      thickness_to_chord                      [-]
      dihedral                                [radians]
      tag                                     <string>
      Segments.*. (optional)
        twist                                 [radians]
        percent_span_location                 [-]  .1 is 10%
        root_chord_percent                    [-]  .1 is 10%
        dihedral_outboard                     [radians]
        sweeps.quarter_chord                  [radians]
        thickness_to_chord                    [-]
    propulsors.turbofan. (optional)
      number_of_engines                       [-]
      engine_length                           [m]
      nacelle_diameter                        [m]
      origin                                  [m] in all three dimension, should have as many origins as engines
      OpenVSP_simple (optional)               <boolean> if False (default) create a flow through nacelle, if True creates a roughly biparabolic shape
    fuselages.fuselage (optional)
      width                                   [m]
      lengths.total                           [m]
      heights.
        maximum                               [m]
        at_quarter_length                     [m]
        at_wing_root_quarter_chord            [m]
        at_three_quarters_length              [m]
      effective_diameter                      [m]
      fineness.nose                           [-] ratio of nose section length to fuselage width
      fineness.tail                           [-] ratio of tail section length to fuselage width
      tag                                     <string>
      OpenVSP_values.  (optional)
        nose.top.angle                        [degrees]
        nose.top.strength                     [-] this determines how much the specified angle influences that shape
        nose.side.angle                       [degrees]
        nose.side.strength                    [-]
        nose.TB_Sym                           <boolean> determines if top angle is mirrored on bottom
        nose.z_pos                            [-] z position of the nose as a percentage of fuselage length (.1 is 10%)
        tail.top.angle                        [degrees]
        tail.top.strength                     [-]
        tail.z_pos (optional, 0.02 default)   [-] z position of the tail as a percentage of fuselage length (.1 is 10%)
    
    Outputs:
    <tag>.vsp3           This is the OpenVSP representation of the aircraft
    Properties Used:
    N/A
    """    
    
    # Reset OpenVSP to avoid including a previous vehicle
    try:
        vsp.ClearVSPModel()
    except NameError:
        print 'VSP import failed'
        return -1
    
    area_tags = dict() # for wetted area assignment
    
    # -------------
    # Wings
    # -------------
    
    for wing in vehicle.wings:
    
        wing_x = wing.origin[0]    
        wing_y = wing.origin[1]
        wing_z = wing.origin[2]
        if wing.symmetric == True:
            span   = wing.spans.projected/2. # span of one side
        else:
            span   = wing.spans.projected
        root_chord = wing.chords.root
        tip_chord  = wing.chords.tip
        sweep      = wing.sweeps.quarter_chord / Units.deg
        sweep_loc  = 0.25
        root_twist = wing.twists.root / Units.deg
        tip_twist  = wing.twists.tip  / Units.deg
        root_tc    = wing.thickness_to_chord 
        tip_tc     = wing.thickness_to_chord 
        dihedral   = wing.dihedral / Units.deg
        
        # Check to see if segments are defined. Get count
        if len(wing.Segments.keys())>0:
            n_segments = len(wing.Segments.keys())
        else:
            n_segments = 0

        # Create the wing
        wing_id = vsp.AddGeom( "WING" )
        vsp.SetGeomName(wing_id, wing.tag)
        area_tags[wing.tag] = ['wings',wing.tag]
            
        # Make names for each section and insert them into the wing if necessary
        x_secs       = []
        x_sec_curves = []
        # n_segments + 2 will create an extra segment if the root segment is 
        # included in the list of segments. This is not used and the tag is
        # removed when the segments are checked for this case.
        for i_segs in xrange(0,n_segments+2):
            x_secs.append('XSec_' + str(i_segs))
            x_sec_curves.append('XSecCurve_' + str(i_segs))

        # Apply the basic characteristics of the wing to root and tip
        if wing.symmetric == False:
            vsp.SetParmVal( wing_id,'Sym_Planar_Flag','Sym',0)
        if wing.vertical == True:
            vsp.SetParmVal( wing_id,'X_Rel_Rotation','XForm',90)     
            
        vsp.SetParmVal( wing_id,'X_Rel_Location','XForm',wing_x)
        vsp.SetParmVal( wing_id,'Y_Rel_Location','XForm',wing_y)
        vsp.SetParmVal( wing_id,'Z_Rel_Location','XForm',wing_z)
        
        # This ensures that the other VSP parameters are driven properly
        vsp.SetDriverGroup( wing_id, 1, vsp.SPAN_WSECT_DRIVER, vsp.ROOTC_WSECT_DRIVER, vsp.TIPC_WSECT_DRIVER )
        
        # Root chord
        vsp.SetParmVal( wing_id,'Root_Chord',x_secs[1],root_chord)
        
        # Sweep of the first section
        vsp.SetParmVal( wing_id,'Sweep',x_secs[1],sweep)
        vsp.SetParmVal( wing_id,'Sweep_Location',x_secs[1],sweep_loc)
        
        # Twists
        if n_segments != 0:
            if wing.Segments[0].percent_span_location == 0.:
                vsp.SetParmVal( wing_id,'Twist',x_secs[0],wing.Segments[0].twist / Units.deg) # root
            else:
                vsp.SetParmVal( wing_id,'Twist',x_secs[0],root_twist) # root
            if wing.Segments[-1].percent_span_location == 1.:
                vsp.SetParmVal( wing_id,'Twist',x_secs[-2],wing.Segments[0].twist / Units.deg) # root
            else:
                vsp.SetParmVal( wing_id,'Twist',x_secs[-2],tip_twist) # root
        else:
            vsp.SetParmVal( wing_id,'Twist',x_secs[0],root_twist) # root
            vsp.SetParmVal( wing_id,'Twist',x_secs[0],tip_twist) # tip
            
            
        # Figure out if there is an airfoil provided
        
        # Airfoils should be in Lednicer format
        # i.e. :
        #
        #EXAMPLE AIRFOIL
        # 3. 3. 
        #
        # 0.0 0.0
        # 0.5 0.1
        # 1.0 0.0
        #
        # 0.0 0.0
        # 0.5 -0.1
        # 1.0 0.0
    
        # Note this will fail silently if airfoil is not in correct format
        # check geometry output
        
        if n_segments==0:
            if len(wing.Airfoil) != 0:
                xsecsurf = vsp.GetXSecSurf(wing_id,0)
                vsp.ChangeXSecShape(xsecsurf,0,vsp.XS_FILE_AIRFOIL)
                vsp.ChangeXSecShape(xsecsurf,1,vsp.XS_FILE_AIRFOIL)
                xsec1 = vsp.GetXSec(xsecsurf,0)
                xsec2 = vsp.GetXSec(xsecsurf,1)
                vsp.ReadFileAirfoil(xsec1,wing.Airfoil['airfoil'].coordinate_file)
                vsp.ReadFileAirfoil(xsec2,wing.Airfoil['airfoil'].coordinate_file)
                vsp.Update()
        else: # The wing airfoil is still used for the root segment if the first added segment does not begin there
            # This could be combined with above, but is left here for clarity
            if (len(wing.Airfoil) != 0) and (wing.Segments[0].percent_span_location!=0.):
                xsecsurf = vsp.GetXSecSurf(wing_id,0)
                vsp.ChangeXSecShape(xsecsurf,0,vsp.XS_FILE_AIRFOIL)
                vsp.ChangeXSecShape(xsecsurf,1,vsp.XS_FILE_AIRFOIL)
                xsec1 = vsp.GetXSec(xsecsurf,0)
                xsec2 = vsp.GetXSec(xsecsurf,1)
                vsp.ReadFileAirfoil(xsec1,wing.Airfoil['airfoil'].coordinate_file)
                vsp.ReadFileAirfoil(xsec2,wing.Airfoil['airfoil'].coordinate_file)
                vsp.Update()
            elif len(wing.Segments[0].Airfoil) != 0:
                xsecsurf = vsp.GetXSecSurf(wing_id,0)
                vsp.ChangeXSecShape(xsecsurf,0,vsp.XS_FILE_AIRFOIL)
                vsp.ChangeXSecShape(xsecsurf,1,vsp.XS_FILE_AIRFOIL)
                xsec1 = vsp.GetXSec(xsecsurf,0)
                xsec2 = vsp.GetXSec(xsecsurf,1)
                vsp.ReadFileAirfoil(xsec1,wing.Segments[0].Airfoil['airfoil'].coordinate_file)
                vsp.ReadFileAirfoil(xsec2,wing.Segments[0].Airfoil['airfoil'].coordinate_file)
                vsp.Update()                
        
        # Thickness to chords
        vsp.SetParmVal( wing_id,'ThickChord','XSecCurve_0',root_tc)
        vsp.SetParmVal( wing_id,'ThickChord','XSecCurve_1',tip_tc)
        
        # Dihedral
        vsp.SetParmVal( wing_id,'Dihedral',x_secs[1],dihedral)
        
        # Span and tip of the section
        if n_segments>1:
            local_span    = span*wing.Segments[0].percent_span_location  
            sec_tip_chord = root_chord*wing.Segments[0].root_chord_percent
            vsp.SetParmVal( wing_id,'Span',x_secs[1],local_span) 
            vsp.SetParmVal( wing_id,'Tip_Chord',x_secs[1],sec_tip_chord)
        else:
            vsp.SetParmVal( wing_id,'Span',x_secs[1],span) 
            
        vsp.Update()
          
        if n_segments>0:
            if wing.Segments[0].percent_span_location==0.:
                x_secs[-1] = [] # remove extra section tag (for clarity)
                segment_0_is_root_flag = True
                adjust = 0 # used for indexing
            else:
                segment_0_is_root_flag = False
                adjust = 1
        else:
            adjust = 1
            
        
        # Loop for the number of segments left over
        for i_segs in xrange(1,n_segments+1):  
            
            if (wing.Segments[i_segs-1] == wing.Segments[-1]) and (wing.Segments[-1].percent_span_location == 1.):
                break
            
            # Unpack
            dihedral_i = wing.Segments[i_segs-1].dihedral_outboard / Units.deg
            chord_i    = root_chord*wing.Segments[i_segs-1].root_chord_percent
            try:
                twist_i    = wing.Segments[i_segs].twist / Units.deg
                no_twist_flag = False
            except:
                no_twist_flag = True
            sweep_i    = wing.Segments[i_segs-1].sweeps.quarter_chord / Units.deg
            tc_i       = wing.Segments[i_segs-1].thickness_to_chord
            
            # Calculate the local span
            if i_segs == n_segments:
                span_i = span*(1 - wing.Segments[i_segs-1].percent_span_location)/np.cos(dihedral_i*Units.deg)
            else:
                span_i = span*(wing.Segments[i_segs].percent_span_location-wing.Segments[i_segs-1].percent_span_location)/np.cos(dihedral_i*Units.deg)                      
            
            # Insert the new wing section with specified airfoil if available
            if len(wing.Segments[i_segs-1].Airfoil) != 0:
                vsp.InsertXSec(wing_id,i_segs-1+adjust,vsp.XS_FILE_AIRFOIL)
                xsecsurf = vsp.GetXSecSurf(wing_id,0)
                xsec = vsp.GetXSec(xsecsurf,i_segs+adjust)
                vsp.ReadFileAirfoil(xsec, wing.Segments[i_segs-1].Airfoil['airfoil'].coordinate_file)                
            else:
                vsp.InsertXSec(wing_id,i_segs-1+adjust,vsp.XS_FOUR_SERIES)
            
            # Set the parms
            vsp.SetParmVal( wing_id,'Span',x_secs[i_segs+adjust],span_i)
            vsp.SetParmVal( wing_id,'Dihedral',x_secs[i_segs+adjust],dihedral_i)
            vsp.SetParmVal( wing_id,'Sweep',x_secs[i_segs+adjust],sweep_i)
            vsp.SetParmVal( wing_id,'Sweep_Location',x_secs[i_segs+adjust],sweep_loc)      
            vsp.SetParmVal( wing_id,'Root_Chord',x_secs[i_segs+adjust],chord_i)
            if not no_twist_flag:
                vsp.SetParmVal( wing_id,'Twist',x_secs[i_segs+adjust],twist_i)
            vsp.SetParmVal( wing_id,'ThickChord',x_sec_curves[i_segs+adjust],tc_i)
            
            if adjust and (i_segs == 1):
                vsp.Update()
                vsp.SetParmVal( wing_id,'Twist',x_secs[1],wing.Segments[i_segs-1].twist / Units.deg)
            
            vsp.Update()
       
        if (n_segments != 0) and (wing.Segments[-1].percent_span_location == 1.):
            tip_chord = root_chord*wing.Segments[-1].root_chord_percent
            vsp.SetParmVal( wing_id,'Tip_Chord',x_secs[n_segments-1+adjust],tip_chord)
            vsp.SetParmVal( wing_id,'ThickChord',x_secs[n_segments-1+adjust],wing.Segments[-1].thickness_to_chord)
            # twist is set in the normal loop
        else:
            vsp.SetParmVal( wing_id,'Tip_Chord',x_secs[-1-(1-adjust)],tip_chord)
            vsp.SetParmVal( wing_id,'Twist',x_secs[-1-(1-adjust)],tip_twist)
            # a single trapezoidal wing is assumed to have constant thickness to chord
        vsp.Update()
        vsp.SetParmVal(wing_id,'CapUMaxOption','EndCap',2.)
        vsp.SetParmVal(wing_id,'CapUMaxStrength','EndCap',1.)
        
        vsp.Update() # to fix problems with chords not matching up
        
        if wing.tag == 'main_wing':
            main_wing_id = wing_id
            
            
    ## Skeleton code for props and pylons can be found in previous commits (~Dec 2016) if desired
    ## This was a place to start and may not still be functional
    
    # -------------
    # Engines
    # -------------
    
    if vehicle.propulsors.has_key('turbofan'):
        
        print 'Warning: no meshing sources are currently implemented for the nacelle'
    
        # Unpack
        turbofan  = vehicle.propulsors.turbofan
        n_engines = turbofan.number_of_engines
        length    = turbofan.engine_length
        width     = turbofan.nacelle_diameter
        origins   = turbofan.origin
        
        # True will make a biconvex body, false will make a flow-through subsonic nacelle
        if turbofan.has_key('OpenVSP_simple'):
            simple_flag = turbofan.OpenVSP_simple
        else:
            simple_flag = False
        
        for ii in xrange(0,int(n_engines)):

            origin = origins[ii]
            
            x = origin[0]
            y = origin[1]
            z = origin[2]
            
            nac_id = vsp.AddGeom( "FUSELAGE")
            vsp.SetGeomName(nac_id, 'turbofan')
            
            # Origin
            vsp.SetParmVal(nac_id,'X_Location','XForm',x)
            vsp.SetParmVal(nac_id,'Y_Location','XForm',y)
            vsp.SetParmVal(nac_id,'Z_Location','XForm',z)
            vsp.SetParmVal(nac_id,'Abs_Or_Relitive_flag','XForm',vsp.ABS) # misspelling from OpenVSP
            vsp.SetParmVal(nac_id,'Origin','XForm',0.5)            
            
            if simple_flag == True:
                vsp.CutXSec(nac_id,3)
                vsp.CutXSec(nac_id,1)
                angle = np.arctan(width/length) / Units.deg
                vsp.SetParmVal(nac_id,"TopLAngle","XSec_0",angle)
                vsp.SetParmVal(nac_id,"TopLAngle","XSec_2",-angle)
                vsp.SetParmVal(nac_id,"AllSym","XSec_0",1)
                vsp.SetParmVal(nac_id,"AllSym","XSec_1",1)
                vsp.SetParmVal(nac_id,"AllSym","XSec_2",1)
                vsp.SetParmVal(nac_id,"Length","Design",length)
                vsp.SetParmVal(nac_id, "Ellipse_Width", "XSecCurve_1", width)
                vsp.SetParmVal(nac_id, "Ellipse_Height", "XSecCurve_1", width)
                
            else:
            
                # Length and overall diameter
                vsp.SetParmVal(nac_id,"Length","Design",length)
                vsp.SetParmVal(nac_id,'OrderPolicy','Design',1.) 
                vsp.SetParmVal(nac_id,'Z_Rotation','XForm',180.)
                
                xsecsurf = vsp.GetXSecSurf(nac_id,0)
                vsp.ChangeXSecShape(xsecsurf,0,vsp.XS_ELLIPSE)
                vsp.Update()
                vsp.SetParmVal(nac_id, "Ellipse_Width", "XSecCurve_0", width-.2)
                vsp.SetParmVal(nac_id, "Ellipse_Width", "XSecCurve_1", width)
                vsp.SetParmVal(nac_id, "Ellipse_Width", "XSecCurve_2", width)
                vsp.SetParmVal(nac_id, "Ellipse_Width", "XSecCurve_3", width)
                vsp.SetParmVal(nac_id, "Ellipse_Height", "XSecCurve_0", width-.2)
                vsp.SetParmVal(nac_id, "Ellipse_Height", "XSecCurve_1", width)
                vsp.SetParmVal(nac_id, "Ellipse_Height", "XSecCurve_2", width)
                vsp.SetParmVal(nac_id, "Ellipse_Height", "XSecCurve_3", width)
            
            vsp.Update()
    
    # -------------
    # Fuselage
    # -------------    
    
    if vehicle.fuselages.has_key('fuselage'):
        # Unpack
        fuselage = vehicle.fuselages.fuselage
        width    = fuselage.width
        length   = fuselage.lengths.total
        hmax     = fuselage.heights.maximum
        height1  = fuselage.heights.at_quarter_length
        height2  = fuselage.heights.at_wing_root_quarter_chord 
        height3  = fuselage.heights.at_three_quarters_length
        effdia   = fuselage.effective_diameter
        n_fine   = fuselage.fineness.nose 
        t_fine   = fuselage.fineness.tail  
        w_ac     = wing.aerodynamic_center
        
        w_origin = vehicle.wings.main_wing.origin
        w_c_4    = vehicle.wings.main_wing.chords.root/4.
        
        # Figure out the location x location of each section, 3 sections, end of nose, wing origin, and start of tail
        
        x1 = n_fine*width/length
        x2 = (w_origin[0]+w_c_4)/length
        x3 = 1-t_fine*width/length
        
        fuse_id = vsp.AddGeom("FUSELAGE") 
        vsp.SetGeomName(fuse_id, fuselage.tag)
        area_tags[fuselage.tag] = ['fuselages',fuselage.tag]
    
        tail_z_pos = 0.02 # default value
        if fuselage.has_key('OpenVSP_values'):
            

            vals = fuselage.OpenVSP_values
            
            # for wave drag testing
            fuselage.OpenVSP_ID = fuse_id
            
            # Nose
            vsp.SetParmVal(fuse_id,"TopLAngle","XSec_0",vals.nose.top.angle)
            vsp.SetParmVal(fuse_id,"TopLStrength","XSec_0",vals.nose.top.strength)
            vsp.SetParmVal(fuse_id,"RightLAngle","XSec_0",vals.nose.side.angle)
            vsp.SetParmVal(fuse_id,"RightLStrength","XSec_0",vals.nose.side.strength)
            vsp.SetParmVal(fuse_id,"TBSym","XSec_0",vals.nose.TB_Sym)
            vsp.SetParmVal(fuse_id,"ZLocPercent","XSec_0",vals.nose.z_pos)
            
            
            # Tail
            vsp.SetParmVal(fuse_id,"TopLAngle","XSec_4",vals.tail.top.angle)
            vsp.SetParmVal(fuse_id,"TopLStrength","XSec_4",vals.tail.top.strength)
            # Below can be enabled if AllSym (below) is removed
            #vsp.SetParmVal(fuse_id,"RightLAngle","XSec_4",vals.tail.side.angle)
            #vsp.SetParmVal(fuse_id,"RightLStrength","XSec_4",vals.tail.side.strength)
            #vsp.SetParmVal(fuse_id,"TBSym","XSec_4",vals.tail.TB_Sym)
            #vsp.SetParmVal(fuse_id,"BottomLAngle","XSec_4",vals.tail.bottom.angle)
            #vsp.SetParmVal(fuse_id,"BottomLStrength","XSec_4",vals.tail.bottom.strength)
            if vals.tail.has_key('z_pos'):
                tail_z_pos = vals.tail.z_pos
            else:
                pass # use above default
                
            vsp.SetParmVal(fuse_id,"AllSym","XSec_4",1)
    
        vsp.SetParmVal(fuse_id,"Length","Design",length)
        vsp.SetParmVal(fuse_id,"Diameter","Design",width)
        vsp.SetParmVal(fuse_id,"XLocPercent","XSec_1",x1)
        vsp.SetParmVal(fuse_id,"XLocPercent","XSec_2",x2)
        vsp.SetParmVal(fuse_id,"XLocPercent","XSec_3",x3)
        vsp.SetParmVal(fuse_id,"ZLocPercent","XSec_4",tail_z_pos)
        vsp.SetParmVal(fuse_id, "Ellipse_Width", "XSecCurve_1", width)
        vsp.SetParmVal(fuse_id, "Ellipse_Width", "XSecCurve_2", width)
        vsp.SetParmVal(fuse_id, "Ellipse_Width", "XSecCurve_3", width)
        vsp.SetParmVal(fuse_id, "Ellipse_Height", "XSecCurve_1", height1);
        vsp.SetParmVal(fuse_id, "Ellipse_Height", "XSecCurve_2", height2);
        vsp.SetParmVal(fuse_id, "Ellipse_Height", "XSecCurve_3", height3);   
    
    # Write the vehicle to the file
    
    vsp.WriteVSPFile(tag + ".vsp3")
    
    return area_tags