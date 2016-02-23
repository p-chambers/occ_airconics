# -*- coding: utf-8 -*-
"""
Created on Fri Dec 18 15:52:58 2015

 LIFTINGSURFACE.PY ============================================================
 This module contains the definition of the class of 3d lifting surfaces.
 This class can be instantiated to generate wings, tailplanes, fins, propeller-
 or rotor blades, etc.

 This is an OCC_AirCONICS file, based on the Rhino 'AirCONICS' plugin
 by A. Sobester: https://github.com/sobester/AirCONICS
 ==============================================================================

@author: pchambers
"""
import numpy as np
import primitives
import AirCONICStools as act

from OCC.gp import gp_Pnt, gp_Trsf, gp_Vec
from OCC.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.GeomAbs import GeomAbs_C2

class LiftingSurface:

    def __init__(self, ApexPoint,
                 SweepFunct,
                 DihedralFunct,
                 TwistFunct,
                 ChordFunct,
                 AirfoilFunct,
                 ChordFactor=1,
                 ScaleFactor=1,
                 OptimizeChordScale=0,
                 LooseSurf=1,
                 SegmentNo=11,
                 TipRequired=False,
                 max_degree=8,
                 continuity=GeomAbs_C2
                 ):
        """
        Parameters
        ----------
        continuity - integer
            the order of continuity i.e. C^0, C^1, C^2...
        """

        self.ApexPoint = gp_Pnt(*ApexPoint)
        self.SweepFunct = SweepFunct
        self.DihedralFunct = DihedralFunct
        self.TwistFunct = TwistFunct
        self.ChordFunct = ChordFunct
        self.AirfoilFunct = AirfoilFunct
        
        self.ChordFactor = ChordFactor
        self.ScaleFactor = ScaleFactor
        
        # TODO: LooseSurface, Segmentnumber and TipRequired:
        self.LooseSurf = LooseSurf
        self.SegmentNo = SegmentNo
        self.TipRequired = TipRequired
        self.OptimizeChordScale = OptimizeChordScale
        self._max_degree = max_degree
        self._Cont = continuity
#        self._CreateConstructionGeometry()
        self.GenerateLiftingSurface()

# TODO: CreateConstructionGeometry from rhino plugin needs migrating to OCC    
#    def _CreateConstructionGeometry(self):
#        self.PCG1 = rs.AddPoint([-100,-100,0])
#        self.PCG2 = rs.AddPoint([-100,100,0])
#        self.PCG3 = rs.AddPoint([100,100,0])
#        self.PCG4 = rs.AddPoint([100,-100,0])
#        self.XoY_Plane = rs.AddSrfPt([self.PCG1, self.PCG2, self.PCG3, self.PCG4])
#        self.PCG5 = rs.AddPoint([3,3,0])
#        self.PCG6 = rs.AddPoint([3,3,100])
#        self.ProjVectorZ = rs.VectorCreate(self.PCG5, self.PCG6)
    
    
    def _GenerateLeadingEdge(self):
        """Epsilon coordinate attached to leading edge defines sweep
         Returns airfoil leading edge points
         """
        SegmentLength = 1.0 / self.SegmentNo
        
#       Array of epsilon at segment midpoints (will evaluate curve here)
        Epsilon_midpoints = np.linspace(SegmentLength/2., 1-SegmentLength/2.,
                                    self.SegmentNo)

#       We are essentially reconstructing a curve from known slopes at 
#       known curve length stations - a sort of Hermite interpolation 
#       without knowing the ordinate values. If SegmentNo -> Inf, the
#       actual slope at each point -> the sweep angle specified by 
#       SweepFunct
        Tilt_array = self.DihedralFunct(Epsilon_midpoints)
        Sweep_array = self.SweepFunct(Epsilon_midpoints)
        
        DeltaXs =  SegmentLength*np.sin(Sweep_array*np.pi/180.)
        DeltaYs = (SegmentLength*np.cos(Tilt_array*np.pi /180.) * 
                                    np.cos(Sweep_array*np.pi/180.))
        DeltaZs = DeltaYs*np.tan(Tilt_array*np.pi/180.)
        
#        Initialise LE coordinate arrays and add first OCC gp_pnt at [0,0,0]:
#        Note: Might be faster to bypass XLE arrays and use local x only
#        XLE = np.zeros(self.SegmentNo + 1)
#        YLE = np.zeros(self.SegmentNo + 1)
#        ZLE = np.zeros(self.SegmentNo + 1)
#        LEPoints = [gp_Pnt(XLE[0], YLE[0], ZLE[0])]
        LEPoints = np.zeros((self.SegmentNo + 1, 3))

#        for i in xrange(self.SegmentNo):
#            XLE[i+1] = XLE[i] + DeltaXs[i]
#            YLE[i+1] = YLE[i] + DeltaYs[i]
#            ZLE[i+1] = ZLE[i] + DeltaZs[i]
#            LEPoints[i+1, :] = XLE[i+1], YLE[i+1], ZLE[i+1]
        Deltas = np.vstack([DeltaXs, DeltaYs, DeltaZs]).T
        LEPoints [1:, :] = np.cumsum(Deltas, axis=0)

        return LEPoints

    def _BuildLS(self, ChordFactor, ScaleFactor):
        # Generates a tentative lifting surface, given the general, nondimensio-
        # nal parameters of the object (variations of chord length, dihedral, etc.)
        # and the two scaling factors.

        LEPoints = self._GenerateLeadingEdge()
        
        Sections = []
        # TODO: These lists are used for when the curve has been smoothed or
        # the loft has failed, neither of which have been implemented yet
#        ProjectedSections = []
#        TEPoints_u = []
#        TEPoints_l = []
        
        Eps = np.linspace(0, 1, self.SegmentNo+1)
        Sections = [self.AirfoilFunct(Eps[i], LEPoints[i], self.ChordFunct,
                                      ChordFactor, self.DihedralFunct,
                                      self.TwistFunct).Curve
                                      for i in xrange(self.SegmentNo+1)]

        self._Sections = Sections   # I used from debugging - remove it?
        
        # TODO: Implement chord projection and Curve start/end points 
        # to rescale smoothed curves and for secondary loft methods

        LS = act.AddSurfaceLoft(Sections, max_degree=self._max_degree, continuity=self._Cont)    # Skip second (gives strange surface)
#        LS = act.Add_Network_Surface(Sections)
        
        # Trying out a different surface fit (LE was wavy)
#        LE = act.points_to_bspline(LEPoints)
#        TE = act.points_to_bspline([Section.GetObject().EndPoint() for Section in Sections])
#        LS = act.Add_Network_Surface([TE, LE, Sections[0], Sections[-1]], initsurf=LS)
#        from OCC.BRepFill import BRepFill_Filling
#        fill = BRepFill_Filling(3)
#        curves = Sections+[LE,TE]
#        fill.LoadInitSurface(LS)
#        fill.Add(make_edge(Sections[0]),1)
#        fill.Add(make_edge(Sections[-1]),1)
#        fill.Add(make_edge(LE),1)
#        fill.Add(make_edge(TE),1)

#        fill.Build()
#        LS = fill.Generated()
        
        if LS==None:
            pass
###############################################################################
            # TODO: backup surface loft is not yet implemented for OCC 
            # Version of Airconics (this is legacy from the Rhino plugin)

#            Failed to fit loft surface. Try another fitting algorithm
#            TECurve_u = rs.AddInterpCurve(TEPoints_u)
#            TECurve_l = rs.AddInterpCurve(TEPoints_l)
#
#            rails = []
#            list.append(rails, TECurve_u)
#            list.append(rails, TECurve_l)
#
#            # Are the first and last curves identical?
#            # AddSweep fails if they are, so if that is the case, one is skipped
#            CDev = rs.CurveDeviation(Sections[0],Sections[-1])
#            if CDev==None:
#                shapes = Sections
#                LS = rs.AddSweep2(rails, shapes, False)
#            else:
#                shapes = Sections[:-1]
#                LS = rs.AddSweep2(rails, shapes, True)
#            
#            rs.DeleteObjects(rails)
#            rs.DeleteObjects([TECurve_u, TECurve_l])
###############################################################################
            
        WingTip = None

        if self.TipRequired:
            pass
            # TODO: Not yet implemented 'TipRequired'
#            TipCurve = Sections[-1]
#            TipCurve = act.AddTEtoOpenAirfoil(TipCurve)
#            WingTip = rs.AddPlanarSrf(TipCurve)
#            rs.DeleteObject(TipCurve)

# TODO: Calculating surface area
#        # Calculate projected area
#        # In some cases the projected sections cannot all be lofted in one go
#        # (it happens when parts of the wing fold back onto themselves), so
#        # we loft them section by section and we compute the area as a sum.
#        LSP_area = 0
#        # Attempt to compute a projected area
#        try:
#            for i, LEP in enumerate(ProjectedSections):
#                if i < len(ProjectedSections)-1:
#                    LSPsegment = rs.AddLoftSrf(ProjectedSections[i:i+2])
#                    SA = rs.SurfaceArea(LSPsegment)
#                    rs.DeleteObject(LSPsegment)
#                    LSP_area = LSP_area + SA[0]
#        except:
#            print "Failed to compute projected area. Using half of surface area instead."
#            LS_area = rs.SurfaceArea(LS)
#            LSP_area = 0.5*LS_area[0]

# TODO: Check bounding box size
#        BB = rs.BoundingBox(LS)
#        if BB:
#            ActualSemiSpan = BB[2].Y - BB[0].Y
#        else:
#            ActualSemiSpan = 0.0

# TODO: Garbage Collection:
#        # Garbage collection
#        rs.DeleteObjects(Sections)
#        try:
#            rs.DeleteObjects(ProjectedSections)
#        except:
#            print "Cleanup: no projected sections to delete"
#        rs.DeleteObjects(LEPoints)
        
        # Scaling
        if self.ScaleFactor != 1:
            Origin = gp_Pnt(0.,0.,0.)
            LS = act.scale_uniformal(LS, Origin, self.ScaleFactor)
            # TODO: Wing tip scaling (TipRequired is not implemented yet)
            if self.TipRequired and WingTip:
                pass
#               WingTip = rs.ScaleObject(WingTip, Origin, ScaleXYZ)
#
#        rs.DeleteObject(Origin)
#
#        ActualSemiSpan = ActualSemiSpan*ScaleFactor
#        LSP_area = LSP_area*ScaleFactor**2.0
#        RootChord = (self.ChordFunct(0)*ChordFactor)*ScaleFactor
#        AR = ((2.0*ActualSemiSpan)**2.0)/(2*LSP_area)
        self.RootChord = (self.ChordFunct(0)*ChordFactor)*ScaleFactor

        # Temporarily set other variables as None until above TODO's are done
        ActualSemiSpan = None
        LSP_area = None
        AR = None
        return LS, ActualSemiSpan, LSP_area, AR, WingTip

    def GenerateLiftingSurface(self):
        """This is the main method of this class. It builds a lifting
        surface (wing, tailplane, etc.) with the given ChordFactor and
        ScaleFactor or an optimized ChordFactor and ScaleFactor, with the
        local search started from the two given values."""
        x0 = [self.ChordFactor, self.ScaleFactor]
        

        # TODO: Optimize chord scale ...
#            if self.OptimizeChordScale:
#                self._CheckOptParCorrectlySpec()
#                self._NormaliseWeightings()
#                self._PrintTargetsAndWeights()
#                print("Optimizing scale factors...")
#                # An iterative local hillclimber type optimizer is needed here. One
#                # option might be SciPy's fmin as below:
#                # x0, fopt, iter, funcalls, warnflag, allvecs = scipy.optimize.fmin(self._LSObjective, x0, retall=True, xtol=0.025, full_output=True)
#                # However, SciPy is not supported on 64-bit Rhino installations, so 
#                # so here we use an alternative: a simple evoltionary optimizer
#                # included with AirCONICS_tools.
#                MaxIter = 50
#                xtol = 0.025
#                deltax = [x0[0]*0.25,x0[1]*0.25]
#                x0, fopt = act.boxevopmin2d(self._LSObjective, x0, deltax, xtol, MaxIter)
#                x0[0] = abs(x0[0])
#                x0[1] = abs(x0[1])
#                print("Optimum chord factor %5.3f, optimum scale factor %5.3f" % (x0[0], x0[1]))

        LS, ActualSemiSpan, LSP_area, AR, WingTip = \
                                                self._BuildLS(x0[0], x0[1])
        
        # Position the wing at the apex:
        vec = gp_Vec(gp_Pnt(0., 0., 0.), self.ApexPoint)
        LS = act.translate_topods_from_vector(LS, vec)  
        
        self.Shape = LS

        return None

    def Rotate(self, axis, degrees):
        """Rotates the LiftingSurface.shape around axis by angle (degrees)

        Parameters
        ----------
        axis - OCC.gp.gp_Ax1
            An opencascade axis type e.g. X axis = OCC.gp.gp_OX()
        degrees - float
            Angle by which to rotate the shape (degrees)
        """
        if self.Shape is None:
            print("Shape not found: No rotation possible")
            return None
        else:
            # Translate the shape:
            self.Shape = act.rotate(self.Shape, axis, degrees)

            # Translate the apex: will come in useful for future?
            self.ApexPoint.Rotate(axis, degrees)

    def Translate_ByApex(self, newapex):
        """Translate the LiftingSurface.shape and its apex point to newapex
        Parameters
        ----------
        newapex - OCC.gp.gp_Pnt or array of float
            The new LE point or X, Y, Z coordinates
        """
        if self.Shape is None:
            print("Shape not found: No rotation possible")
            return None
        else:
            if type(newapex) is not gp_Pnt:
                newapex = gp_Pnt(*newapex)

            # Create the Translation vector:
            vec = gp_Vec(self.ApexPoint, newapex)

            # Store the new apex point:
            self.ApexPoint = newapex

            # Translate surface:
            self.Shape = act.translate_topods_from_vector(self.Shape, vec)
