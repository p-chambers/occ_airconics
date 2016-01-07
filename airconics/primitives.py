# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 13:31:35 2015

@author: pchambers
"""
import os
from pkg_resources import resource_string, resource_exists
import numpy as np

from OCC.Geom import Handle_Geom_BSplineCurve_DownCast
#from OCC.Geom2dAPI import Geom2dAPI_PointsToBSpline
from OCC.gp import gp_Pnt, gp_Pnt2d, gp_Pln, gp_Dir, gp_Vec, gp_OX, gp_OY
#from OCC.FairCurve import FairCurve_MinimalVariation

import CRMfoil
import airconics.AirCONICStools as act


class Airfoil:
    """ Lifting surface section primitive class """

    def __init__(self,
                 LeadingEdgePoint,
                 ChordLength=1,
                 Rotation=0,
                 Twist=0,
                 SeligProfile=False,
                 Naca4Profile=False,
                 Naca5Profile=False,
                 CRMProfile=False,
                 CRM_Epsilon=0.,
                 EnforceSharpTE=False):
        """Class Constructor: creates and returns an Airfoil instance
        Parameters
        ----------
        LeadingEdgePoint - array of float (,3)
            (x, y, z) origin of the airfoil LE
        ChordLength - scalar
            Length of the airfoil chord
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
        CRM_Profile - bool
            If true, airfoil profile will be interpolated from Common Research
            Model (CRM). Must also declare 'CRMEpsilon' variable.
        CRM_Epsilon - float
            Spanwise fraction between 0 and 1 to interpolate profile from CRM
        EnforceSharpTE - bool
            Enforces sharp trailing edge (NACA airfoils only)
        Notes
        -----
        Either SeligProfile, Naca4Profile or Naca5Profile should be given to
        this function. Specifying more than one profile will give an error.
        
        """
        Profiles = [SeligProfile, Naca4Profile, Naca5Profile, CRMProfile]
#        Input checks:
        assert(any(Profiles)),\
            "No Profile specified. See help(Airfoil)"
        assert(sum([1 for prof in Profiles if prof])),\
            "Ambiguous airfoil: More than one profile has been specified"
        if CRMProfile:
            assert(CRM_Epsilon >= 0 and CRM_Epsilon <= 1), \
                "No Spanwise interpolation Epsilon given: See help(Airfoil)"
            self.CRM_Epsilon = CRM_Epsilon
        
        self._LE = LeadingEdgePoint
        self._ChordLength = ChordLength
        self._Rotation = Rotation
        self._Twist = Twist
        self._EnforceSharpTE = EnforceSharpTE
        self._make_airfoil(SeligProfile, Naca4Profile, Naca5Profile,
                           CRMProfile)


    def _make_airfoil(self, SeligProfile, Naca4Profile, Naca5Profile,
                      CRMProfile):
        """Selects airfoil 'add' function based on Profile specified """
        if SeligProfile:
            self.SeligProfile = SeligProfile
            self.AddAirfoilFromSeligFile(SeligProfile)
        elif Naca4Profile:
            self.Naca4Profile = Naca4Profile
            self.AddNACA4(Naca4Profile)
        elif Naca5Profile:
            raise NotImplementedError("This function is not yet\
                implemented for Naca 5 digit profiles")
        elif CRMProfile:
            self.AddCRMLinear()
        else:
            raise TypeError("Unknown airfoil type: see help(Airfoil)")

    def _fitAirfoiltoPoints(self, x, z):
        """ Fits an OCC curve to airfoil x, z points
        Parameters
        ----------
        x - array
            airfoil curve x points
        z - array
            airfoil curve z points
        Returns
        -------
        spline_2d - 
        
        """
        N = len(x)
        pnts = [gp_Pnt(x[i], 0., z[i]) for i in xrange(N)]
#        plan = gp_Pln(gp_Pnt(0., 0., 0.), gp_Dir(0., 1., 0.))  # XZ plane

        # use the array to create a spline describing the airfoil section
#        spline = GeomAPI_PointsToBSpline(pt_array)#,
                                             # N-1,  # order min
                                             # N)   # order max
#        spline = geomapi.To3d(spline_2d.Curve(), plan)

        Curve = act.points_to_bspline(pnts)
        return Curve

              
    def _AirfoilPointsSeligFormat(self, SeligProfile):
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
        x = np.zeros(N)
        z = np.zeros(N)
        for i, line in enumerate(data):
            vals = line.split()    # vals[0] = x coord, vals[1] = y coord
            x[i] = float(vals[0])
            z[i] = float(vals[1])
        return x*self._ChordLength, z*self._ChordLength

    def _NACA4cambercurve(self, MaxCamberLocTenthChord, MaxCamberPercChord):
        """ Generates the camber curve of a NACA 4-digit airfoil
        Paramters
        ---------
        
        Returns
        -------
        
        """
        # Using the original notation of Jacobs et al.(1933)
        xmc     = MaxCamberLocTenthChord /10.0;
        zcammax = MaxCamberPercChord    /100.0;
        
        # Protect against division by zero on airfoils like NACA0012
        if xmc==0:
            xmc = 0.2 

        # Sampling the chord line
        ChordCoord, NCosPoints = act.coslin(xmc)
        
        # Compute the two sections of the camber curve and its slope
#        zcam = []
#        dzcamdx = []
        cos_pts = ChordCoord[0:NCosPoints]
        lin_pts = ChordCoord[NCosPoints:]
        
        zcam = np.hstack(( (zcammax/(xmc ** 2)) * (2*xmc*cos_pts - cos_pts**2), 
                           (zcammax/((1-xmc)**2)) * \
                             (1-2*xmc+2*xmc*lin_pts-(lin_pts ** 2)) ))
        
        dzcamdx = np.hstack(( (zcammax/xmc ** 2)*(2*xmc - 2*cos_pts),
                              (zcammax/(1-xmc) ** 2)*(2*xmc - 2*lin_pts) ))

        return ChordCoord, zcam, dzcamdx
        
    def _NACA4halfthickness(self, ChordCoord, MaxThicknessPercChord):
        """Given  a set of ordinates  and  a  maximum thickness value
        (expressed in units of chord) it computes the NACA 4-digit
        half-thickness distribution.  The abscissas must be in the
        range [0,1]."""

        # Max thickness in units of chord
        tmax = MaxThicknessPercChord / 100.0

        # Coefficient tweak to close off the trailing edge if required
        a0 =  0.2969/0.2
        a1 = -0.1260/0.2
        a2 = -0.3516/0.2
        a3 =  0.2843/0.2
        a4 = -0.1015/0.2
        # The highest order term could be fudged to make t(1) = 0, thus
        # producing a sharp trailing edge (NACA4s by definition have a finite
        # thickness TE). However, this is probably better enforced by removing
        # a wedge from the coordinate sets (a generic method). Still, this
        # might be a NACA-specific alternative:
        # t_at_one = a0+a1+a2+a3+a4
        # a4 = a4 - t_at_one

        # Half-thickness polynomial
        t = tmax * (a0*ChordCoord**0.5 + a1*ChordCoord + a2*ChordCoord**2.0 +
                    a3*ChordCoord**3.0 + a4*ChordCoord**4.0)
        return t
        
    def _camberplusthickness(self, ChordCoord, zcam, dzcamdx, t):
        """Internal function. Adds a thickness distribution to a specified 
        camber line. The slope is an input here, because it is usually possible
        to compute it analytically at the same time as the curve itself is
        computed."""

        # Theta angle (slope of the camber curve)
        Theta = np.arctan(dzcamdx)

        xu = ChordCoord - t*np.sin(Theta)
        xl = ChordCoord + t*np.sin(Theta)
        zu = zcam + t*np.cos(Theta)
        zl = zcam - t*np.cos(Theta)
            
        # Correct small abscissa positioning errors in case of sharp TE
        if self._EnforceSharpTE:
            xu[-1] = ChordCoord[-1]
            xl[-1] = ChordCoord[-1]

        return xu, zu, xl, zl, Theta

    def _mergesurfaces(self, xu, zu, xl, zl):
        """Combine the upper and lower surfaces into one"""
        
        if self._EnforceSharpTE:
            # Remove wedge to sharpen trailing edge if needed
#            pass
            zu -= xu*zu[-1]
            zl -= xl*zl[-1]
            
        # Combine upper and lower (Top surface reversed from right to left)
        x = np.hstack((xu[::-1], xl[1:]))   # Remove duplicate LE point 
        z = np.hstack((zu[::-1], zl[1:]))
        return x, z

    def _NACA4digitPnts(self, MaxCamberPercChord, MaxCamberLocTenthChord,
                        MaxThicknessPercChord):
        """Generates a set of points that define a NACA 4-digit airfoil
        Parameters
        ----------
        
        Returns
        -------
        """

        ChordCoord, zcam, dzcamdx = \
            self._NACA4cambercurve(MaxCamberLocTenthChord, MaxCamberPercChord)

        t = self._NACA4halfthickness(ChordCoord, MaxThicknessPercChord)

        xu, zu, xl, zl, Theta = self._camberplusthickness(ChordCoord, zcam,
                                                          dzcamdx, t)
#        Scale points:
        xu *= self._ChordLength
        zu *= self._ChordLength
        xl *= self._ChordLength
        zl *= self._ChordLength
        # Leading edge radius
        RLE = 1.1019*(MaxThicknessPercChord/100.0)**2.0
        
        x,z = self._mergesurfaces(xu,zu,xl,zl)
        
        return x, z, xu, zu, xl, zl, RLE   
        
    def _TransformAirfoil(self):
        """Given a normal airfoil, nose in origin, chord along 
        x axis, applies rotations, translation and (soon) smoothing
        """
        # TODO: Smoothing
#        for i in range(self.SmoothingIterations):
#            rs.FairCurve(self.Curve)
        Curve = self.Curve.GetObject()

#        Rotations - Note that direction is opposite to Rhino
#        Dihedral:
        if self._Rotation:
            # self.Curve = Handle_Geom_BSplineCurve_DownCast(Curve.Rotate(A1))
            Curve.Rotate(gp_OX(), np.radians(self._Rotation))
#            act.rotate(Curve, gp_OX(), -self._Rotation)

#        Twist:
        if self._Twist:
            Curve.Rotate(gp_OY(), -np.radians(self._Twist))
        
#        Translation:
        self.Curve = Handle_Geom_BSplineCurve_DownCast(Curve.Translated(
                                                        gp_Vec(*self._LE))
                                                        )
        return None

    def AddAirfoilFromSeligFile(self, SeligProfile, Smoothing=1):
        """Adds an airfoil generated by fitting a NURBS curve to a set 
        of points whose coordinates are given in a Selig formatted file
        Parameters
        ----------
        
        Returns
        -------
        
        """
        assert(type(SeligProfile) == str), "Selig Profile must be a string"
        assert(SeligProfile != ''), "No Selig Profile given (string)"
        x, z = self._AirfoilPointsSeligFormat(SeligProfile) 
        self.Curve = self._fitAirfoiltoPoints(x, z)
        self._TransformAirfoil()
        return None
        
    def AddNACA4(self, Naca4Profile, Smoothing=1):
        """Adds a NACA 4 digit airfoil to the current document
        Parameters
        ----------
        
        Returns
        -------
        """
        assert(type(Naca4Profile) == str), "Selig Profile must be a string"
        assert(len(Naca4Profile)==4), \
            "Invalid Naca4 '{}': should be 4 digit string".format(Naca4Profile)
        
        MaxCamberPercChord     = int(Naca4Profile[0])
        MaxCamberLocTenthChord = int(Naca4Profile[1])
        MaxThicknessPercChord  = int(Naca4Profile[2:])
        
        x, z, xu, zu, xl, zl, RLE = self._NACA4digitPnts(MaxCamberPercChord,
                                                         MaxCamberLocTenthChord,
                                                         MaxThicknessPercChord)
                                                         
        self.Curve = self._fitAirfoiltoPoints(x, z)
#        if 'Smoothing' in locals():
#            self.SmoothingIterations = Smoothing
        self._TransformAirfoil()
        return None
        
    def AddCRMLinear(self, Smoothing=1):
        """Linearly interpolate airfoil curve from CRM"""
        x, z = CRMfoil.CRMlinear(self.CRM_Epsilon)
        x *= self._ChordLength
        z *= self._ChordLength
        self.Curve = self._fitAirfoiltoPoints(x, z)
        # TODO: Smoothing..
#        if 'Smoothing' in locals():
#            self.SmoothingIterations = Smoothing
        self._TransformAirfoil()
        return None
        
#        def Delete(self):
#            """Use to remove shape"""
#            # TODO: Delete function
#            return None


