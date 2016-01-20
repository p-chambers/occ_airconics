# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 11:27:08 2016

# A function for generating a parametric fuselage external surface (outer mould
# line) model. For a description of the parameterisation, see the article:
# A. Sobester, "'Self-designing' Parametric Geometries", AIAA SciTech 2015,
# Orlando, FL.

@author: pchambers
"""
# ==============================================================================
# AirCONICS
# Aircraft CONfiguration through Integrated Cross-disciplinary Scripting 
# version 0.2
# Andras Sobester, 2015.
# Bug reports to a.sobester@soton.ac.uk or @ASobester please.
# ==============================================================================
import AirCONICStools as act
import numpy as np

from OCC.gp import gp_Pnt, gp_Vec, gp_Pln, gp_Dir


class Fuselage:
    
    def __init__(self, NoseLengthRatio=0.182,
                 TailLengthRatio=0.293,
                 Scaling=[55.902, 55.902, 55.902],
                 NoseCoordinates=[0., 0., 0],
                 CylindricalMidSection=False,
                 SimplificationReqd = False,
                 Maxi_attempt=6):
        """AirCONICS Fuselage class: builds a parameterised instance of
        an aircraft fuselage
        Parameters
        ----------
        NoseLengthRatio - Scalar
            The fraction of nose to fuselage length (default 0.182)
        TailLengthRatio - Scalar
            The fraction of tail to fuselage length (default 0.293)
        Scaling - array, length 3
            (x, y, z) scaling factor
        NoseCoordinates - array of float
            Location of nose apex
        CylindricalMidSection - bool
            If True, fuselage will have a cylindrical midsection
        SimplificationReqd - bool
            TODO
        MaxFittingAtempts - integer
            Maximum number of times to attempt to fit surface to guide curves
        """
        self.NoseLengthRatio = NoseLengthRatio
        self.TailLengthRatio = TailLengthRatio
        self.Scaling = Scaling
        self.NoseCoordinates = NoseCoordinates
        self.CylindricalMidSection = CylindricalMidSection
        self.SimplificationReqd = SimplificationReqd
        
        self._BuildFuselageOML(Maxi_attempt)

        
    def _AirlinerFuselagePlanView(self, NoseLengthRatio, TailLengthRatio):
        """Internal function. Defines the control
        polygons of the fuselage in side view"""

        kN = NoseLengthRatio / 0.182
        tN = TailLengthRatio / 0.293

        PlanPort = np.array([[0,                  0,     0],
                             [0*kN,              -0.1,   0],
                             [0.332*kN,          -0.395, 0],
                             [1.250*kN,          -0.810, 0],
                             [2.517*kN,          -1.074, 0],
                             [4*kN,              -1.15,  0],
                             [4*kN,              -1.15,  0],
                             # Parallel sided section here
                             [22-(22-15.55)*tN,  -1.15,   0],
                             [22-(22-15.55)*tN,  -1.15,   0],
                             [22-(22-16.428)*tN, -1.126,  0],
                             [22-(22-20.3362)*tN,-0.483,  0],
                             [22,                -0.0987, 0]])
#        Scale:
        PlanPort *= 2.541
    
        NoseEndX = 4*kN*2.541
        TailStartX = (22-(22-15.55)*tN)*2.541
    
        return PlanPort, NoseEndX, TailStartX
        
    def _AirlinerFuselageSideView(self, NoseLengthRatio, TailLengthRatio):
        """Internal function. Defines the control
        polygons of the fuselage in side view"""
        kN = NoseLengthRatio / 0.182
        tN = TailLengthRatio / 0.293

        # The upper contour control points
        # of the fuselage in side view
        AFSVUpper = np.array([[0,                 0, 0],
                              [0,                 0, 0.3],
                              [1.395*kN,          0, 1.547],
                              [4*kN,              0, 1.686],
                              [4*kN,              0, 1.686],
                              # parallel section here
                              [22-(22-15.55)*tN,  0, 1.686],
                              [22-(22-15.55)*tN,  0, 1.686],
                              [22-(22-19.195)*tN, 0, 1.549],
                              [22,                0, 0.904]])
#        Scale:
        AFSVUpper *= 2.541

        # The lower contour control points
        # of the fuselage in side view
        AFSVLower = np.array([[0,                0,  0],
                              [0,                0, -0.3],
                              [0.947*kN,         0, -0.517],
                              [4*kN,             0, -0.654],
                              [4*kN,             0, -0.654],
                              # Parallel sides section
                              [22-(22-15.55)*tN, 0, -0.654],
                              [22-(22-15.55)*tN, 0, -0.654],
                              # Tailstrike slope section
                              [22-(22-18.787)*tN,0, -0.256],
                              [22,               0,  0.694]])
        AFSVLower *= 2.541

        return AFSVUpper, AFSVLower

    def _FuselageLongitudinalGuideCurves(self, NoseLengthRatio, TailLengthRatio):
        """Internal function. Defines the four longitudinal curves that outline the 
        fuselage (outer mould line).""" 

        FSVU, FSVL = self._AirlinerFuselageSideView(NoseLengthRatio,
                                                    TailLengthRatio)
        FSVUCurve = act.points_to_BezierCurve(FSVU)
        FSVLCurve = act.points_to_BezierCurve(FSVL)
        
        AFPVPort, NoseEndX, TailStartX = \
            self._AirlinerFuselagePlanView(NoseLengthRatio, TailLengthRatio)
        
        # Generate plan view
        PlanPortCurve = act.points_to_BezierCurve(AFPVPort)
        
        # TODO: How wide is the fuselage (use bounding box)
        # Note: THIS DOESNT WORK AS OCC BOUNDING BOX ROUTINES INCLUDE CURVE 
        # POLES. MAY BE ABLE TO WORKAROUND WITH TRIANGULATION, BUT FOR NOW
        # I WILL USE THE INPUT CURVE POINTS
#        H_PlanPortCurve = PlanPortCurve.GetHandle()            # Get handle of curve      
#        PP_Edge = act.make_edge(H_PlanPortCurve)        
#        (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = act.ObjectsExtents([PP_Edge])
        (Xmin,Ymin,Zmin) = np.min(AFPVPort, axis=0)
        (Xmax,Ymax,Zmax) = np.max(AFPVPort, axis=0)
        print(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax)
#        Store the visualisable bounding box:
#        self._bbox = act.BBox_FromExtents(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax)
        
        

        
#        # TODO: Generate a (slightly wider) projection surface
           # Could just average points then add curve? No meancurve in OCC?
#        FSVMeanCurve = rs.MeanCurve(FSVUCurve, FSVLCurve)
#        RuleLinePort      = rs.AddLine((0,0,0),(0,-1.1*abs(Ymax-Ymin),0))
#        FSVMCEP = rs.CurveEndPoint(FSVMeanCurve)
#        AftLoftEdgePort      = rs.CopyObject(RuleLinePort,     FSVMCEP)
#        ParallelLoftEdgePort      = rs.CopyObject(FSVMeanCurve,(0,-1.1*abs(Ymax-Ymin),0))
#        LSPort      = rs.AddSweep2((FSVMeanCurve,ParallelLoftEdgePort     ),(RuleLinePort,     AftLoftEdgePort     ))
#
#       Create the Side view mean curve & Edge
        FSVMean = (FSVU+FSVL) / 2.        
        FSVMeanCurve = act.points_to_BezierCurve(FSVMean)
        FSVMeanEdge = act.make_edge(FSVMeanCurve.GetHandle())
        
        RuleLinePort = act.make_edge(gp_Pnt(0.,0.,0.),
                                     gp_Pnt(0., -1.1*abs(Ymax-Ymin), 0.))
        FSVMCEP = FSVMeanCurve.EndPoint()
        MoveVec = gp_Vec(gp_Pnt(0, 0, 0), FSVMCEP)
        AftLoftEdgePort = act.translate_topods_from_vector(RuleLinePort,
                                                           MoveVec,
                                                           copy=True)
        # Make copy of the mean curve                                                    
        MoveVec = gp_Vec(gp_Pnt(0, 0, 0), gp_Pnt(0, -1.1*abs(Ymax-Ymin), 0))
        ParallelLoftEdgePort = act.translate_topods_from_vector(
                                                        FSVMeanEdge,
                                                        MoveVec,
                                                        copy=True)
#        LSPort = 
        
#        # Project the plan view onto the mean surface
#        PortCurve      = rs.ProjectCurveToSurface(PlanPortCurve     , LSPort     ,(0,0,100))
#    
#        # TODO: House-keeping
#        rs.DeleteObjects([LSPort,PlanPortCurve,ParallelLoftEdgePort,RuleLinePort,AftLoftEdgePort])
#    
        # TODO: Tidy up the mean curve
#        # Tidy up the mean curve. This is necessary for a smooth result and removing
#        # it can render the algorithm unstable. However, FitCurve itself may sometimes
#        # be slightly unstable.
#        FLength = abs(Xmax-Xmin) # establish a reference length
#        PortCurveSimplified      = rs.FitCurve(PortCurve,     distance_tolerance = FLength*0.001)
#        StarboardCurveSimplified = act.MirrorObjectXZ(PortCurveSimplified)
#        
#        rs.DeleteObject(PortCurve)
#        
#        # Compute the actual end points of the longitudinal curves
        # TODO: Compute end points of curves
#        (Xmin,Ymin,Zmin,Xmax1,Ymax,Zmax) = act.ObjectsExtents(StarboardCurveSimplified)
#        (Xmin,Ymin,Zmin,Xmax2,Ymax,Zmax) = act.ObjectsExtents(PortCurveSimplified)
#        (Xmin,Ymin,Zmin,Xmax3,Ymax,Zmax) = act.ObjectsExtents(FSVUCurve)
#        (Xmin,Ymin,Zmin,Xmax4,Ymax,Zmax) = act.ObjectsExtents(FSVLCurve)
#        EndX = min([Xmax1,Xmax2,Xmax3,Xmax4])
        StarboardCurveSimplified = None
        PortCurveSimplified = PlanPortCurve
        FSVMeanCurve = None
        EndX = Xmax        #This is not correct: just trying to get it working
        return StarboardCurveSimplified, PortCurveSimplified, FSVUCurve, FSVLCurve, FSVMeanCurve, NoseEndX, TailStartX, EndX

    def _BuildFuselageOML(self, Max_attempt):
        """Builds the Fuselage outer mould line"""
    

        NetworkSrfSettings = np.array([[35, 20, 15, 5, 20],
                                       [35, 30, 15, 5, 20],
                                       [35, 20, 15, 2, 20],
                                       [30, 30, 15, 2, 20],
                                       [30, 20, 15, 2, 20],
                                       [25, 20, 15, 2, 20],
                                       [20, 20, 15, 2, 20],
                                       [15, 20, 15, 2, 20]])
        StarboardCurve, PortCurve, FSVUCurve, FSVLCurve, FSVMeanCurve, \
            NoseEndX, TailStartX, EndX =                               \
            self._FuselageLongitudinalGuideCurves(self.NoseLengthRatio,
                                                  self.TailLengthRatio)
    
        self.FSVUCurve = FSVUCurve
        self.FSVLCurve = FSVLCurve
        self.PortCurve = PortCurve
        
        # Note: I should remove this later as it overwrites input
        Max_attempt = 1
        
        i_attempt = -1
        while i_attempt <= Max_attempt:
#    
            i_attempt = i_attempt + 1 
#            
            # Construct array of cross section definition frames
            
            SX0 = 0
            SX1 = 0.04*NoseEndX
            SX2 = SX1 + 0.25*NoseEndX
            SX3 = NoseEndX
            SX4 = TailStartX
            SX5 = EndX

            Step01, Step12, Step23, Step34, Step45 = \
                NetworkSrfSettings[i_attempt]
            
#            print "Attempting network surface fit with network density setup ", NetworkSrfSettings[i_attempt][:]
            Stations01 = np.linspace(SX0, SX1, max([Step01, 2]))
            Stations12 = np.linspace(SX1, SX2, max([Step12, 2]))
            Stations23 = np.linspace(SX2, SX3, max([Step23, 2]))
            Stations34 = np.linspace(SX3, SX4, max([Step34, 2]))
            Stations45 = np.linspace(SX4, SX5, max([Step45, 2]))
        
            StationRange = np.hstack([Stations01[:-1], Stations12[:-1],
                                     Stations23[:-1], Stations34[:-1],
                                     Stations45])
           
            C = []
            FirstTime = True

            self._SectionPlanes = []            
            
            for XStation in StationRange:
                # Create plane normal to x direction
                P = gp_Pln(gp_Pnt(XStation, 0, 0), gp_Dir(gp_Vec(1, 0, 0)))
                # Make into a face so I can check:
                self._SectionPlanes.append(act.make_face(P))              
                
#                P = rs.PlaneFromPoints((XStation,0,0),(XStation,1,0),(XStation,0,1))
#                IP1 = rs.PlaneCurveIntersection(P,StarboardCurve)
#                IP2 = rs.PlaneCurveIntersection(P,FSVUCurve)
#                IP3 = rs.PlaneCurveIntersection(P,PortCurve)
#                IP4 = rs.PlaneCurveIntersection(P,FSVLCurve)
#                IPcentre = rs.PlaneCurveIntersection(P,FSVMeanCurve)
#                IPoint1 = rs.AddPoint(IP1[0][1])
#                IPoint2 = rs.AddPoint(IP2[0][1])
#                IPoint3 = rs.AddPoint(IP3[0][1])
#                IPoint4 = rs.AddPoint(IP4[0][1])
#                IPointCentre = rs.AddPoint(IPcentre[0][1])
#                PseudoDiameter = abs(IP4[0][1].Z-IP2[0][1].Z)
#                if CylindricalMidSection and NoseEndX < XStation < TailStartX:
#                # Ensure that the parallel section of the fuselage is cylindrical
#                # if CylindricalMidSection is True
#                    print "Enforcing circularity in the central section..."
#                    if FirstTime:
#                        PseudoRadius = PseudoDiameter/2
#                        FirstTime = False
#                    Pc = rs.PointCoordinates(IPointCentre)
#                    P1 = P2 = P3 = Pc
#                    P1 = rs.PointAdd(P1,(0,PseudoRadius,0))
#                    P2 = rs.PointAdd(P2,(0,0,PseudoRadius))
#                    P3 = rs.PointAdd(P3,(0,-PseudoRadius,0))
#                    c = rs.AddCircle3Pt(P1, P2, P3)
#                else:
#                    c = rs.AddInterpCurve([IPoint1,IPoint2,IPoint3,IPoint4,IPoint1],knotstyle=3)
#                    # Once CSec is implemented in Rhino Python, this could be replaced
#                rs.DeleteObjects([IPoint1,IPoint2,IPoint3,IPoint4,IPointCentre])
#                list.append(C,c)
#        
#            # Fit fuselage external surface
#            CurveNet = []
#            for c in C[1:]:
#                list.append(CurveNet,c)
#            list.append(CurveNet, FSVUCurve)
#            list.append(CurveNet, PortCurve)
#            list.append(CurveNet, FSVLCurve)
#            list.append(CurveNet, StarboardCurve)
#            FuselageOMLSurf = rs.AddNetworkSrf(CurveNet)
#            rs.DeleteObjects(C)
#            
#            if not(FuselageOMLSurf==None):
#                print "Network surface fit succesful on attempt ", i_attempt+1 
#                i_attempt = Maxi_attempt+1 # Force an exit from 'while'
#    
#        # If all attempts at fitting a network surface failed, we attempt a Sweep2
#        if FuselageOMLSurf==None:
#            print "Failed to fit network surface to the external shape of the fuselage"
#            print "Attempting alternative fitting method, quality likely to be low..."
#    
#            try:
#                FuselageOMLSurf = rs.AddSweep2([FSVUCurve,FSVLCurve],C[:])
#            except:
#                FuselageOMLSurf = False
#    
#            SimplificationReqd = True # Enforce simplification
#            if not(FuselageOMLSurf):
#                print "Alternative fitting method failed too. Out of ideas."
#    
#        if FuselageOMLSurf and SimplificationReqd:
#            rs.UnselectAllObjects()
#            rs.SelectObject(FuselageOMLSurf)
#            ToleranceStr = str(0.0005*EndX)
#            print "Smoothing..."
#            rs.Command("FitSrf " + ToleranceStr)
#            rs.UnselectAllObjects()
#    
#        # Compute the stern point coordinates of the fuselage
#        Pu = rs.CurveEndPoint(FSVUCurve)
#        Pl = rs.CurveEndPoint(FSVLCurve)
#        SternPoint = [Pu.X, Pu.Y, 0.5*(Pu.Z+Pl.Z)]
#    
#        rs.DeleteObjects([FSVUCurve,FSVLCurve,PortCurve,StarboardCurve,FSVMeanCurve])
#    
#        return FuselageOMLSurf, SternPoint






###############################################################################








#def CockpitWindowContours(Height = 1.620, Depth = 5):
#    P1 = [0.000,0.076,Height-1.620+2.194]
#    P2 = [0.000,0.852,Height-1.620+2.290]
#    P3 = [0.000,0.904,Height+0.037]
#    P4 = [0.000,0.076,Height]
#    CWC1 = rs.AddPolyline([P1,P2,P3,P4,P1])
#    rs.SelectObject(CWC1)
#    rs.Command("_FilletCorners 0.08 ")
#
#    P1 = [0.000,0.951,Height-1.620+2.289]
#    P2 = [0.000,1.343,Height-1.620+2.224]
#    P3 = [0.000,1.634,Height-1.620+1.773]
#    P4 = [0.000,1.557,Height-1.620+1.588]
#    P5 = [0.000,1.027,Height-1.620+1.671]
#    CWC2 = rs.AddPolyline([P1,P2,P3,P4,P5,P1])
#    rs.SelectObject(CWC2)
#    rs.Command("_FilletCorners 0.08 ")
#
#    CWC3 = act.MirrorObjectXZ(CWC1)
#    CWC4 = act.MirrorObjectXZ(CWC2)
#    
#    ExtPathId = rs.AddLine([0,0,0],[Depth, 0, 0])
#    
#    CWC1s = rs.ExtrudeCurve(CWC1, ExtPathId)
#    CWC2s = rs.ExtrudeCurve(CWC2, ExtPathId)
#    CWC3s = rs.ExtrudeCurve(CWC3, ExtPathId)
#    CWC4s = rs.ExtrudeCurve(CWC4, ExtPathId)
#
#    rs.DeleteObjects([CWC1, CWC2, CWC3, CWC4, ExtPathId])
#
#    return CWC1s, CWC2s, CWC3s, CWC4s
#
#
#def WindowContour(WinCenter):
#    P1 = [WinCenter[0], 0, WinCenter[1] + 0.468/2]
#    P2 = [WinCenter[0] + 0.272/2, 0, WinCenter[1]]
#    P3 = [WinCenter[0], 0, WinCenter[1] - 0.468/2]
#    P4 = [WinCenter[0] - 0.272/2, 0, WinCenter[1]]
#
#    WCurveU = rs.AddInterpCurve([P4, P1, P2], start_tangent = [0, 0, 2.5], 
#    end_tangent = [0, 0, -2.5])
#    WCurveL = rs.AddInterpCurve([P2, P3, P4], start_tangent = [0, 0, -2.5], 
#    end_tangent = [0, 0, 2.5])
#    
#    WCurve = rs.JoinCurves([WCurveU, WCurveL], delete_input=True)
#    return WCurve
#
#def MakeWindow(FuselageSrf, Xwc, Zwc):
#    WinCenter = [Xwc, Zwc]
#    WCurve = WindowContour(WinCenter)
#    
#    ExtPathStbd = rs.AddLine([0,0,0],[0,10,0])
#    ExtPathPort = rs.AddLine([0,0,0],[0,-10,0])
#    
#    TubeStbd = rs.ExtrudeCurve(WCurve, ExtPathStbd)
#    FuselageSrf, WinStbd = rs.SplitBrep(FuselageSrf, TubeStbd, delete_input=True)
#    TubePort = rs.ExtrudeCurve(WCurve, ExtPathPort)
#    FuselageSrf, WinPort = rs.SplitBrep(FuselageSrf, TubePort, delete_input=True)
#
#    rs.DeleteObjects([TubeStbd, TubePort, ExtPathStbd, ExtPathPort, WCurve])
#
#    return WinStbd, WinPort, FuselageSrf






#def FuselageOML(NoseLengthRatio = 0.182, TailLengthRatio = 0.293, Scaling = [55.902, 55.902, 55.902], NoseCoordinates = [0,0,0], CylindricalMidSection = False, SimplificationReqd = False):
# Instantiates a parametric fuselage outer mould line (OML) geometry for a given
# set of design variables.
#    FuselageOMLSurf, SternPoint = 
#    if not(FuselageOMLSurf) or FuselageOMLSurf is None:
#        return
#
#    ScalingF = [0,0,0]
#    ScalingF[0] = Scaling[0]/55.902
#    ScalingF[1] = Scaling[1]/55.902
#    ScalingF[2] = Scaling[2]/55.902
#
#    # Overall scaling
#    FuselageOMLSurf = act.ScaleObjectWorld000(FuselageOMLSurf, ScalingF)


    # A few other ways of performing the scaling...
    # Variant one: this depends on the current CPlane!
    # FuselageOMLSurf = rs.ScaleObject(FuselageOMLSurf, (0,0,0), Scaling)
    
    # Variant two: define plane in World coordinates
    #P = rs.PlaneFromFrame((0,0,0),(1,0,0),(0,1,0))
    #TransfMatrix = Rhino.Geometry.Transform.Scale(P, Scaling[0], Scaling[1], Scaling[2])
    #FuselageOMLSurf = rs.TransformObjects(FuselageOMLSurf, TransfMatrix)

    # Variant three: World coordinate system based scaling
    #xform = rs.XformScale(Scaling)
    #FuselageOMLSurf = rs.TransformObjects(FuselageOMLSurf, xform)

#    SternPoint[0] = SternPoint[0]*ScalingF[0]
#    SternPoint[1] = SternPoint[1]*ScalingF[1]
#    SternPoint[2] = SternPoint[2]*ScalingF[2]
#
#    # Positioning
#    MoveVec = rs.VectorCreate(NoseCoordinates, [0,0,0])
#    FuselageOMLSurf = rs.MoveObject(FuselageOMLSurf, MoveVec)
#    SternPoint[0] = SternPoint[0]+NoseCoordinates[0]
#    SternPoint[1] = SternPoint[1]+NoseCoordinates[1]
#    SternPoint[2] = SternPoint[2]+NoseCoordinates[2]
#    
#    return FuselageOMLSurf, SternPoint


if __name__ == '__main__':
    # The defaults will yield a fuselage geometry similar to that of the 
    # Boeing 787-8.
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    Fus = Fuselage()
    # Another example: for a fuselage shape similar to that of the Airbus A380
    # comment out the line above and uncomment the line below:
#    FuselageOML(NoseLengthRatio = 0.182, TailLengthRatio = 0.293, 
#    Scaling = [70.4, 67.36, 80.1], 
#    NoseCoordinates = [0,0,0], 
#    CylindricalMidSection = False, 
#    SimplificationReqd = False)
    
#    display.DisplayShape(Fus.FSVUCurve, update=True)
#    display.DisplayShape(Fus.FSVLCurve, update=True)
    display.DisplayShape(Fus.PortCurve, update=True)
    
    for face in Fus._SectionPlanes:
        display.DisplayShape(face, update=True)
    start_display()