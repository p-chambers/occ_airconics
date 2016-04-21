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
from airconics.base import AirconicsShape

from OCC.gp import gp_Pnt, gp_Vec, gp_Pln, gp_Dir, gp_Ax2
from OCC.Geom import Handle_Geom_BSplineCurve, Geom_Plane
from OCC.GeomAbs import GeomAbs_C2
from OCC.TopoDS import topods


class Fuselage(AirconicsShape):
    def __init__(self, NoseLengthRatio=0.182,
                 TailLengthRatio=0.293,
                 Scaling=[55.902, 55.902, 55.902],
                 NoseCoordinates=[0., 0., 0],
                 CylindricalMidSection=False,
                 SimplificationReqd=False,
                 Maxi_attempt=5):
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

        Notes
        -----
        - Geometry building is done on initialisation of a Fuselage instance.
        It is therefore not expected that users will do this through the
        BuildFuselageOML function
        """
        super(Fuselage, self).__init__(NoseLengthRatio=NoseLengthRatio,
                                       TailLengthRatio=TailLengthRatio,
                                       Scaling=Scaling,
                                       NoseCoordinates=NoseCoordinates,
                                       CylindricalMidSection=CylindricalMidSection,
                                       SimplificationReqd=SimplificationReqd)

        self._BuildFuselageOML(Maxi_attempt)
        self._TransformOML()

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

    def _FuselageLongitudinalGuideCurves(self, NoseLengthRatio,
                                         TailLengthRatio):
        """Internal function. Defines the four longitudinal curves that outline
        the fuselage (outer mould line)."""

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
        # POLES. MAY BE ABLE TO WORKAROUND WITH TRIANGULATION
#        H_PlanPortCurve = PlanPortCurve.GetHandle()     # Get handle of curve
#        PP_Edge = act.make_edge(H_PlanPortCurve)
#        (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = act.ObjectsExtents([PP_Edge])
        (Xmin, Ymin, Zmin) = np.min(AFPVPort, axis=0)
        (Xmax, Ymax, Zmax) = np.max(AFPVPort, axis=0)


#        # TODO: Generate a (slightly wider) projection surface
#        > This is legacy from Rhino
#        FSVMeanCurve = rs.MeanCurve(FSVUCurve, FSVLCurve)
#        RuleLinePort      = rs.AddLine((0,0,0),(0,-1.1*abs(Ymax-Ymin),0))
#        FSVMCEP = rs.CurveEndPoint(FSVMeanCurve)
#        AftLoftEdgePort      = rs.CopyObject(RuleLinePort,     FSVMCEP)
#        ParallelLoftEdgePort      = rs.CopyObject(FSVMeanCurve,(0,-1.1*abs(Ymax-Ymin),0))
#        LSPort      = rs.AddSweep2((FSVMeanCurve,ParallelLoftEdgePort     ),(RuleLinePort,     AftLoftEdgePort     ))
#
#       Mean Curve: This is wrong! But no mean curve found in OCC
        FSVMean = (FSVU+FSVL)/2.

        FSVMeanCurve = act.points_to_BezierCurve(FSVMean)
        FSVMeanEdge = act.make_edge(FSVMeanCurve.GetHandle())
        self._MeanEdge = FSVMeanEdge
        RuleLinePort = act.make_edge(gp_Pnt(0., 0., 0.),
                                     gp_Pnt(0., -1.1*abs(Ymax-Ymin), 0.))
        FSVMCEP = FSVMeanCurve.EndPoint()
        MoveVec = gp_Vec(gp_Pnt(0, 0, 0), FSVMCEP)
        AftLoftEdgePort = topods.Edge(
            act.translate_topods_from_vector(RuleLinePort, MoveVec, copy=True))

#         Make copy of the mean curve
        MoveVec = gp_Vec(gp_Pnt(0, 0, 0), gp_Pnt(0, -1.1*abs(Ymax-Ymin), 0))

#          Didnt need this to construct Port Edge of the loft surface in OCC
#         (Legacy from Rhino) - may experiment with this behaviour later:
#        ParallelLoftEdgePort = topods.Edge(act.translate_topods_from_vector(
#                                                        FSVMeanEdge,
#                                                        MoveVec,
#                                                        copy=True))
#         Mean Surface:
#        LSPort_edges = [RuleLinePort, FSVMeanEdge, AftLoftEdgePort,
#                                                        ParallelLoftEdgePort]
        spine = act.make_wire(FSVMeanEdge)
        section1 = act.make_wire(RuleLinePort)
        section2 = act.make_wire(AftLoftEdgePort)
#        support = act.make_wire(ParallelLoftEdgePort)
        LSPort = act.make_pipe_shell(spine, [section1, section2])
        self._LSPort = LSPort

#        # Project the plan view onto the mean surface
        PortCurveSimplified = \
            act.project_curve_to_surface(PlanPortCurve, LSPort,
                                         gp_Dir(0, 0, 100))

#        # TODO: House-keeping
#        DeleteObjects([LSPort,PlanPortCurve,ParallelLoftEdgePort,RuleLinePort,
#                       AftLoftEdgePort])

        # TODO: Tidy up the mean curve
#         Tidy up the mean curve. This is necessary for a smooth result and
#        removing it can render the algorithm unstable. However, FitCurve
#        itself may sometimes be slightly unstable.
#        FLength = abs(Xmax-Xmin) # establish a reference length
#        PortCurveSimplified      = rs.FitCurve(PortCurve,
#                                        distance_tolerance = FLength*0.001)
#        StarboardCurveSimplified = act.MirrorObjectXZ(PortCurveSimplified)
#
#        rs.DeleteObject(PortCurve)
#
#        # Compute the actual end points of the longitudinal curves
        # TODO: Compute end points of curves (Needed when mean curve has been
#                    Changed by fitting curve)
#        (Xmin,Ymin,Zmin,Xmax1,Ymax,Zmax) =\
#            act.ObjectsExtents(StarboardCurveSimplified)
#        (Xmin,Ymin,Zmin,Xmax2,Ymax,Zmax) =\
#            act.ObjectsExtents(PortCurveSimplified)
#        (Xmin,Ymin,Zmin,Xmax3,Ymax,Zmax) = act.ObjectsExtents(FSVUCurve)
#        (Xmin,Ymin,Zmin,Xmax4,Ymax,Zmax) = act.ObjectsExtents(FSVLCurve)
#        EndX = min([Xmax1,Xmax2,Xmax3,Xmax4])

#        PortCurveSimplified = PlanPortCurve

        # Seems easiest to mirror portcurve with handles?
        h = Handle_Geom_BSplineCurve()
        mirror_ax2 = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
        c = PortCurveSimplified.Copy()
        c.GetObject().Mirror(mirror_ax2)
        h2 = h.DownCast(c)
        StarboardCurve = h2.GetObject()

        EndX = Xmax        # This is not correct: just trying to get it working
        return (StarboardCurve, PortCurveSimplified, FSVUCurve, FSVLCurve,
                FSVMeanCurve, NoseEndX, TailStartX, EndX)

    def _BuildFuselageOML(self, Max_attempt):
        """Builds the Fuselage outer mould line
        Notes
        -----
        It is not expected that users will interact with this directly. Use
        the Fuslage class initialisation fuction instead
        """
        NetworkSrfSettings = np.array([[35, 20, 15, 15, 20],
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
        # Compute the stern point coordinates of the fuselage
        Pu = FSVUCurve.EndPoint()
        Pl = FSVLCurve.EndPoint()
        self.SternPoint = gp_Pnt(Pu.X(), Pu.Y(), 0.5*(Pu.Z()+Pl.Z()))
        Pu = FSVUCurve.StartPoint()
        Pl = FSVLCurve.StartPoint()
        self.BowPoint = gp_Pnt(Pu.X(), Pu.Y(), 0.5*(Pu.Z()+Pl.Z()))

        i_attempt = 0
        while i_attempt < Max_attempt:
            i_attempt = i_attempt + 1
            print("Surface fit attempt ", i_attempt)

            # Construct array of cross section definition frames
            SX0 = 0
            SX1 = 0.04*NoseEndX
            SX2 = SX1 + 0.25*NoseEndX
            SX3 = NoseEndX
            SX4 = TailStartX
            SX5 = EndX

            Step01, Step12, Step23, Step34, Step45 = \
                NetworkSrfSettings[i_attempt-1]

            print("Attempting thrusections surface fit with network density\
                setup ", NetworkSrfSettings[i_attempt][:])
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

            for i, XStation in enumerate(StationRange[1:]):
                # Create plane normal to x direction
                P = Geom_Plane(gp_Pln(gp_Pnt(XStation, 0, 0),
                                      gp_Dir(gp_Vec(1, 0, 0))))
                # Make into a face for visualisation/debugging
                try:
                    IPoint2 = act.points_from_intersection(P, FSVUCurve)
                    IPoint3 = act.points_from_intersection(P, PortCurve)
                    IPoint4 = act.points_from_intersection(P, FSVLCurve)
                    IPoint1 = act.points_from_intersection(P, StarboardCurve)

                    IPointCentre = act.points_from_intersection(P,
                                                                FSVMeanCurve)
                except RuntimeError:
                    print("Intersection Points at Section X={} Not Found"
                          .format(XStation))
                    print("Skipping this plane location")
                    continue

                PseudoDiameter = abs(IPoint4.Z()-IPoint2.Z())
                if self.CylindricalMidSection and\
                        NoseEndX < XStation < TailStartX:
                    print "Enforcing circularity in the central section..."
                    if FirstTime:
                        PseudoRadius = PseudoDiameter / 2.
                        FirstTime = False
                    PseudoRadius = PseudoDiameter / 2.
                    # Note: Add Circle with radius PseudoRadius at Pc
                    from OCC.GC import GC_MakeCircle
                    c = GC_MakeCircle(gp_Ax2(IPointCentre, gp_Dir(1, 0, 0)),
                                      PseudoRadius).Value()

                else:
                    # Set the tangents at each point for interpolation:
                    # assume that these are solely in 1 axis as points lie
                    # extremities of an elliptical shape
                    tangents = np.array([[0, -1,  0],
                                         [0,  0, -1],
                                         [0,  1,  0],
                                         [0,  0,  1]])
                    c = act.points_to_bspline(
                        [IPoint2, IPoint3, IPoint4, IPoint1],
                        periodic=True, scale=False,
                        tangents=tangents)

                C.append(c)

#             Fit fuselage external surface
            sections = [act.make_wire(act.make_edge(curve)) for curve in C]
            guides = ([FSVUCurve.GetHandle(), PortCurve.GetHandle(),
                       FSVLCurve.GetHandle(), StarboardCurve.GetHandle()])
            guides = [act.make_wire(act.make_edge(guide)) for guide in guides]
            self._Lguides = guides
            self._Csections = sections
            self._NoseVertex = act.make_vertex(self.BowPoint)
            try:
                OMLSurf = \
                    act.AddSurfaceLoft(C, first_vertex=self._NoseVertex,
                                       continuity=GeomAbs_C2)
            except:
                OMLSurf = None

            if OMLSurf is not None:
                print("Network surface fit succesful on attempt {}"
                      .format(i_attempt))
                self.AddComponent(OMLSurf, 'OML')
                return None

#         If all attempts at fitting a network surface failed, we attempt a
#            Pipe Shell:
        if OMLSurf is None:
            print("Failed to fit network surface to the external shape of the\
                fuselage")
            print("Attempting alternative fitting method,\
                quality likely to be low...")

            try:
                OMLSurf = act.make_pipe_shell(C)
                self.AddComponent(OMLSurf, 'OML')
            except:
                raise(RuntimeError, "Could not produce a valid OML surface")

#    Note: The following is the last resort surface fit from Rhino Airconics
#                And currently is not available in OCC Airconics:

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

        # Shouldnt get here
        return None

    def _TransformOML(self):
        """Use parameters defined in self to scale and translate the fuselage
        """
        ScalingF = [0, 0, 0]
        ScalingF[0] = self.Scaling[0] / 55.902
        ScalingF[1] = self.Scaling[1] / 55.902
        ScalingF[2] = self.Scaling[2] / 55.902
#
#        # Overall scaling and translation:
        MoveVec = self.NoseCoordinates
        self.TransformComponents_Nonuniformal(ScalingF, MoveVec)

#        SternPoint[0] = SternPoint[0]*ScalingF[0]
#        SternPoint[1] = SternPoint[1]*ScalingF[1]
#        SternPoint[2] = SternPoint[2]*ScalingF[2]
#
#        # Positioning
#        SternPoint[0] = SternPoint[0]+NoseCoordinates[0]
#        SternPoint[1] = SternPoint[1]+NoseCoordinates[1]
#        SternPoint[2] = SternPoint[2]+NoseCoordinates[2]

###############################################################################

if __name__ == '__main__':
    # The defaults will yield a fuselage geometry similar to that of the
    # Boeing 787-8.
    from OCC.Display.SimpleGui import init_display
    display, start_display, add_menu, add_function_to_menu = init_display()

    Fus = Fuselage(NoseLengthRatio=0.182,
                   TailLengthRatio=0.293,
                   Scaling=[55.902, 55.902, 55.902],
                   NoseCoordinates=[0., 0., 0],
                   CylindricalMidSection=False,
                   Maxi_attempt=5)

    # Create plane to check symmetry:
#    P = gp_Pln(gp_Pnt(0, 0, 0),
#                          gp_Dir(gp_Vec(0, 1, 0)))
#    Fsym = act.make_face(P, -10, 10, 0, 100)
#    display.DisplayShape(Fsym, update=True)

#    Display Fuselage:
    Fus.Display(display)

    for section in Fus._Lguides:
        display.DisplayShape(section, color='Black')
    for support in Fus._Csections:
        display.DisplayShape(support, color='Blue')

    start_display()
