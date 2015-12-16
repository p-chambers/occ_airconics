# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 13:31:35 2015

@author: pchambers
"""
import os
from pkg_resources import resource_string, resource_exists
import numpy as np

from OCC.Geom2dAPI import Geom2dAPI_PointsToBSpline
from OCC.gp import gp_Pnt, gp_Pnt2d, gp_Pln, gp_Dir
from OCC.GeomAPI import geomapi

from airconics.AirCONICStools import point2d_list_to_TColgp_Array1OfPnt2d


class Airfoil:
    """ Lifting surface section primitive class """

    def __init__(self,
                 LeadingEdgePoint,
                 ChordLength,
                 Rotation=0,
                 Twist=0,
                 SeligProfile=False,
                 Naca4Profile=False,
                 Naca5Profile=False,
                 EnforceSharpTE=False):
        """Class Constructor: creates and returns an Airfoil instance
        Parameters
        ----------
        LeadingEdgePoint - array of float (,3)
            (x, y, z) origin of the airfoil LE
        ChordLength - 
            #TODO
        Rotation - 
            #TODO
        Twist - 
            #TODO
        SeligProfile - string
            Name of the Selig airfoil curve points
        NACA4Profile - string
            Name of the airfoil in NACA 4 format   
        NACA5Profile - string
            Name of the airfoil in NACA 5 format
        EnforceSharpTE - bool
            #TODO
        Notes
        -----
        Either SeligProfile, Naca4Profile or Naca5Profile should be given to
        this function. Specifying more than one profile will give an error.
        
        """
        Profiles = [SeligProfile, Naca4Profile, Naca5Profile]
#        Input checks:
        assert(any(Profiles)),\
            "No Profile specified. See help(Airfoil)"
        assert(sum([1 for prof in Profiles if prof])),\
            "Ambiguous airfoil: More than one profile has been specified"
        
        self._LeadingEdgePoint = LeadingEdgePoint
        self._ChordLength = ChordLength
        self._Rotation = Rotation
        self._Twist = Twist
#        self.EnforceSharpTE = EnforceSharpTE
        self.shape, self._ChordLength = self._make_airfoil(SeligProfile, 
                                            Naca4Profile, Naca5Profile)


    def _make_airfoil(self, SeligProfile, Naca4Profile, Naca5Profile):
        """Selects airfoil 'add' function based on Profile specified """
        if SeligProfile:
            return self._AddAirfoilFromSeligFile(SeligProfile)
        elif Naca4Profile:
            raise NotImplementedError("Oops, This function is not yet\
                implemented for Naca 4 digit profiles")
        elif Naca5Profile:
            raise NotImplementedError("Oops, This function is not yet\
                implemented for Naca 5 digit profiles")
        else:
            raise TypeError("Unknown airfoil type: see help(Airfoil)")

        # _fitAirfoitoPoints is deprecated: slows down computation.
        # Migrated to 
#    def _fitAirfoiltoPoints(self, x, z): 
#        """ Fits an OCC curve to airfoil x, z points
#        Parameters
#        ----------
#        x - array
#            airfoil curve x points
#        z - array
#            airfoil curve z points
#        Returns
#        -------
#        spline - 
#        
#        """
#        N = len(x)
#        # Note: not sure why the points need to be defined as (z, x) here rather 
#        # than (x, z) as pythonocc example suggests it should be (x,z):
#        section_pts_2d = [gp_Pnt2d(z[i],x[i]) for i in xrange(N)]
#        pt_array = point2d_list_to_TColgp_Array1OfPnt2d(section_pts_2d)
#        plan = gp_Pln(gp_Pnt(0., 0., 0.), gp_Dir(0., 1., 0.))  # XZ plane
#
#        # use the array to create a spline describing the airfoil section
#        spline_2d = Geom2dAPI_PointsToBSpline(pt_array,
#                                              N-1,  # order min
#                                              N)   # order max
#        spline = geomapi.To3d(spline_2d.Curve(), plan)
#        return spline

              
    def _BsplineFromSeligAirfoil(self, SeligProfile):
        """Extracts airfoil coordinates from a file
        
        Assumes input selig files are specified in the Selig format, i.e., 
        header line, followed by x column, z column, from upper trailing edge 
        to lower trailing edge.
        Parameters
        ----------
        FileNameWithPath - string
            
        Returns
        -------
        x - array of float
            x coordinates of airfoil curve
        
        z - array of float
            z coordinates of airfoil curve
        """
        res_pkg = 'airconics.coord_seligFmt'
        SeligProfile = SeligProfile + '.dat'
        assert(resource_exists(res_pkg, SeligProfile)),\
            "Airfoil database for {} not found.".format(SeligProfile)

        data = resource_string(res_pkg, SeligProfile)
        data = data.split('\r\n')[1:-1]
        N = len(data)
        section_pts_2d = []
        for i, line in enumerate(data):
            vals = line.split()    #vals[0] = x coord, vals[1] = y coord
            section_pts_2d.append(gp_Pnt2d(float(vals[0]), float(vals[1])))
            # Note: not sure why the points need to be defined as (z, x) here rather 
            # than (x, z) as pythonocc example suggests it should be (x,z):            

        pt_array = point2d_list_to_TColgp_Array1OfPnt2d(section_pts_2d)
        plan = gp_Pln(gp_Pnt(0., 0., 0.), gp_Dir(0., 0., 1.))  # XZ plane

        # use the array to create a spline describing the airfoil section
        spline_2d = Geom2dAPI_PointsToBSpline(pt_array,
                                              N-1,  # order min
                                              N)   # order max
#        spline = geomapi.To3d(spline_2d.Curve(), plan)
        return spline_2d.Curve()            

                
    def _AddAirfoilFromSeligFile(self, SeligProfile, Smoothing=1):
        """Adds an airfoil generated by fitting a NURBS curve to a set 
        of points whose coordinates are given in a Selig formatted file
        Parameters
        ----------
        
        Returns
        -------
        
        """
        assert(SeligProfile != ''), "No Selig Profile given (string)"
        C = self._BsplineFromSeligAirfoil(SeligProfile)
#        C = self._fitAirfoiltoPoints(x, z)  #Removed for speed!
        Chrd = 1    #TODO: chord transform, smoothing
#            if 'Smoothing' in locals():
#                self.SmoothingIterations = Smoothing
#                C, Chrd = self._TransformAirfoil(C)
        return C, Chrd