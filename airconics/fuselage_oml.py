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
from six.moves import range
from . import AirCONICStools as act
import numpy as np
from .base import AirconicsShape

from OCC.gp import gp_Pnt, gp_Vec, gp_Pln, gp_Dir, gp_Ax2
from OCC.Geom import Handle_Geom_BSplineCurve, Geom_Plane
from OCC.GeomAbs import GeomAbs_C2
from OCC.TopoDS import topods
from OCC.GC import GC_MakeCircle

import logging
log = logging.getLogger(__name__)


class Fuselage(AirconicsShape):
    """AirCONICS Fuselage class: builds a parameterised instance of
    an aircraft fuselage

    Parameters
    ----------
    NoseLengthRatio : Scalar
        The fraction of nose to fuselage length (default 0.182)

    TailLengthRatio : Scalar
        The fraction of tail to fuselage length (default 0.293)

    Scaling : array, length 3
        (x, y, z) scaling factor

    NoseCoordinates : array of float
        Location of nose apex

    CylindricalMidSection : bool
        If True, fuselage will have a cylindrical midsection

    SimplificationReqd : bool
        If true, the geometry will be significantly simplified using a
        truncated ellipsoid nose, cylinder body and tail cone. This is slightly
        different to the behaviour in airconics, as this will be used by
        default if True, rather than as a backup if the bspline fitting failed.

    MaxFittingAtempts : integer
        Maximum number of times to attempt to fit surface to guide curves

    construct_geometry : bool
        If true, Build method will be called on construction

    Notes
    -----
    Geometry building is done on initialisation of a Fuselage instance.
    It is therefore not expected that users will do this through the
    BuildFuselageOML function
    """

    def __init__(self, NoseLengthRatio=0.182,
                 TailLengthRatio=0.293,
                 Scaling=[55.902, 55.902, 55.902],
                 NoseCoordinates=[0., 0., 0],
                 CylindricalMidSection=False,
                 SimplificationReqd=False,
                 Maxi_attempt=5,
                 construct_geometry=True,
                 ):

        # Keywords args passed to super's init are added as attributes, except
        # construct geometry
        super(Fuselage, self).__init__(NoseLengthRatio=NoseLengthRatio,
                                       TailLengthRatio=TailLengthRatio,
                                       Scaling=Scaling,
                                       NoseCoordinates=NoseCoordinates,
                                       CylindricalMidSection=CylindricalMidSection,
                                       SimplificationReqd=SimplificationReqd,
                                       Max_attempt=Maxi_attempt,
                                       construct_geometry=construct_geometry)

    def Build(self):
        """Overrides the AirconicsShape empty Build method.

        Calls BuildFuselageOML, which has been maintained for older versions.
        """
        super(Fuselage, self).Build()
        if self.SimplificationReqd:
            self.BuildFuselageSimplified()
        else:
            self.BuildFuselageOML(self.Max_attempt)
        # Note: TransformOML also moves the stern and bow points
        self.TransformOML()

    def AirlinerFuselagePlanView(self, NoseLengthRatio, TailLengthRatio):
        """Internal function. Defines the control
        polygons of the fuselage in side view"""

        kN=NoseLengthRatio / 0.182
        tN=TailLengthRatio / 0.293

        PlanPort=np.array([[0, 0, 0],
                             [0 * kN, -0.1, 0],
                             [0.332 * kN, -0.395, 0],
                             [1.250 * kN, -0.810, 0],
                             [2.517 * kN, -1.074, 0],
                             [4 * kN, -1.15, 0],
                             [4 * kN, -1.15, 0],
                             # Parallel sided section here
                             [22 - (22 - 15.55) * tN, -1.15, 0],
                             [22 - (22 - 15.55) * tN, -1.15, 0],
                             [22 - (22 - 16.428) * tN, -1.126, 0],
                             [22 - (22 - 20.3362) * tN, -0.483, 0],
                             [22, -0.0987, 0]])
#        Scale:
        PlanPort *= 2.541

        NoseEndX=4 * kN * 2.541
        TailStartX=(22 - (22 - 15.55) * tN) * 2.541

        return PlanPort, NoseEndX, TailStartX

    def AirlinerFuselageSideView(self, NoseLengthRatio, TailLengthRatio):
        """Internal function. Defines the control
        polygons of the fuselage in side view"""
        kN=NoseLengthRatio / 0.182
        tN=TailLengthRatio / 0.293

        # The upper contour control points
        # of the fuselage in side view
        AFSVUpper=np.array([[0, 0, 0],
                              [0, 0, 0.3],
                              [1.395 * kN, 0, 1.547],
                              [4 * kN, 0, 1.686],
                              [4 * kN, 0, 1.686],
                              # parallel section here
                              [22 - (22 - 15.55) * tN, 0, 1.686],
                              [22 - (22 - 15.55) * tN, 0, 1.686],
                              [22 - (22 - 19.195) * tN, 0, 1.549],
                              [22, 0, 0.904]])
#        Scale:
        AFSVUpper *= 2.541

        # The lower contour control points
        # of the fuselage in side view
        AFSVLower=np.array([[0, 0, 0],
                              [0, 0, -0.3],
                              [0.947 * kN, 0, -0.517],
                              [4 * kN, 0, -0.654],
                              [4 * kN, 0, -0.654],
                              # Parallel sides section
                              [22 - (22 - 15.55) * tN, 0, -0.654],
                              [22 - (22 - 15.55) * tN, 0, -0.654],
                              # Tailstrike slope section
                              [22 - (22 - 18.787) * tN, 0, -0.256],
                              [22, 0, 0.694]])
        AFSVLower *= 2.541

        return AFSVUpper, AFSVLower

    def FuselageLongitudinalGuideCurves(self, NoseLengthRatio,
                                        TailLengthRatio):
        """Internal function. Defines the four longitudinal curves that outline
        the fuselage (outer mould line)."""

        FSVU, FSVL=self.AirlinerFuselageSideView(NoseLengthRatio,
                                                   TailLengthRatio)
        FSVUCurve=act.points_to_BezierCurve(FSVU)
        FSVLCurve=act.points_to_BezierCurve(FSVL)

        AFPVPort, NoseEndX, TailStartX=self.AirlinerFuselagePlanView(
            NoseLengthRatio, TailLengthRatio)

        # Generate plan view
        PlanPortCurve=act.points_to_BezierCurve(AFPVPort).GetHandle()

        # TODO: How wide is the fuselage (use bounding box)
        # Note: THIS DOESNT WORK AS OCC BOUNDING BOX ROUTINES INCLUDE CURVE
        # POLES. MAY BE ABLE TO WORKAROUND WITH TRIANGULATION
#        H_PlanPortCurve = PlanPortCurve.GetHandle()     # Get handle of curve
#        PP_Edge = act.make_edge(H_PlanPortCurve)
#        (Xmin,Ymin,Zmin,Xmax,Ymax,Zmax) = act.ObjectsExtents([PP_Edge])
        (Xmin, Ymin, Zmin)=np.min(AFPVPort, axis=0)
        (Xmax, Ymax, Zmax)=np.max(AFPVPort, axis=0)


#        # TODO: Generate a (slightly wider) projection surface
#        > This is legacy from Rhino
#        FSVMeanCurve = rs.MeanCurve(FSVUCurve, FSVLCurve)
#        RuleLinePort      = rs.AddLine((0,0,0),(0,-1.1*abs(Ymax-Ymin),0))
#        FSVMCEP = rs.CurveEndPoint(FSVMeanCurve)
#        AftLoftEdgePort      = rs.CopyObject(RuleLinePort,     FSVMCEP)
#        ParallelLoftEdgePort      = rs.CopyObject(FSVMeanCurve,(0,-1.1*abs(Ymax-Ymin),0))
#        LSPort      = rs.AddSweep2((FSVMeanCurve,ParallelLoftEdgePort     ),(RuleLinePort,     AftLoftEdgePort     ))
#
        FSVMean=(FSVU + FSVL) / 2.

        FSVMeanCurve=act.points_to_BezierCurve(FSVMean)
        FSVMeanEdge=act.make_edge(FSVMeanCurve.GetHandle())
        self._MeanEdge=FSVMeanEdge
        RuleLinePort=act.make_edge(gp_Pnt(0., 0., 0.),
                                     gp_Pnt(0., -1.1 * abs(Ymax - Ymin), 0.))
        FSVMCEP=FSVMeanCurve.EndPoint()
        MoveVec=gp_Vec(gp_Pnt(0, 0, 0), FSVMCEP)
        AftLoftEdgePort=topods.Edge(
            act.translate_topods_from_vector(RuleLinePort, MoveVec, copy=True))

#         Make copy of the mean curve
        MoveVec=gp_Vec(gp_Pnt(0, 0, 0),
                         gp_Pnt(0, - 1.1 * abs(Ymax - Ymin), 0))

#          Didnt need this to construct Port Edge of the loft surface in OCC
#         (Legacy from Rhino) - may experiment with this behaviour later:
#        ParallelLoftEdgePort = topods.Edge(act.translate_topods_from_vector(
#                                                        FSVMeanEdge,
#                                                        MoveVec,
#                                                        copy=True))
#         Mean Surface:
#        LSPort_edges = [RuleLinePort, FSVMeanEdge, AftLoftEdgePort,
#                                                        ParallelLoftEdgePort]
        spine=act.make_wire(FSVMeanEdge)
        section1=act.make_wire(RuleLinePort)
        section2=act.make_wire(AftLoftEdgePort)
#        support = act.make_wire(ParallelLoftEdgePort)
        LSPort=act.make_pipe_shell(spine, [section1, section2])
        self._LSPort=LSPort

        # Project the plan view onto the mean surface: had to move the
        # Port curve here as projection was not working for certain inputs
        PlanPortCurve.GetObject().Translate(gp_Vec(0, 0, -10))

        HPortCurve=act.project_curve_to_surface(PlanPortCurve, LSPort,
                                                  gp_Dir(0, 0, 100))
        PortCurve=HPortCurve.GetObject()
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
        h=Handle_Geom_BSplineCurve()
        mirror_ax2=gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0))
        c=PortCurve.Copy()
        c.GetObject().Mirror(mirror_ax2)
        HStarboardCurve=h.DownCast(c)
#        StarboardCurve = hSt.GetObject()

        EndX=Xmax        # This is not correct: just trying to get it working
        return (HStarboardCurve, HPortCurve, FSVUCurve, FSVLCurve,
                FSVMeanCurve, NoseEndX, TailStartX, EndX)

    def BuildFuselageSimplified(self):
        """Builds the Fuselage outer mould line
        Notes
        -----
        It is not expected that users will interact with this directly. Use
        the Fuslage class initialisation fuction instead
        """
        NetworkSrfSettings=np.array([5, 5, 5, 2, 2])

        FSVU, FSVL=self.AirlinerFuselageSideView(self.NoseLengthRatio,
                                                   self.TailLengthRatio)
        FSVUCurve=act.points_to_BezierCurve(FSVU)
        FSVLCurve=act.points_to_BezierCurve(FSVL)

        FSVMean=(FSVU + FSVL) / 2.

        FSVMeanCurve=act.points_to_BezierCurve(FSVMean)

        kN=self.NoseLengthRatio / 0.182
        tN=self.TailLengthRatio / 0.293

        NoseEndX=4 * kN * 2.541
        TailStartX=(22 - (22 - 15.55) * tN) * 2.541
        EndX=FSVMeanCurve.EndPoint().X()

        # Compute the stern point coordinates of the fuselage
        Pu=FSVUCurve.EndPoint()
        Pl=FSVLCurve.EndPoint()
        self.SternPoint=gp_Pnt(Pu.X(), Pu.Y(), 0.5 * (Pu.Z() + Pl.Z()))
        Pu=FSVUCurve.StartPoint()
        Pl=FSVLCurve.StartPoint()
        self.BowPoint=gp_Pnt(Pu.X(), Pu.Y(), 0.5 * (Pu.Z() + Pl.Z()))

        # Construct array of cross section definition frames
        SX0=0
        SX1=0.04 * NoseEndX
        SX2=SX1 + 0.25 * NoseEndX
        SX3=NoseEndX
        SX4=TailStartX
        SX5=EndX

        Step01, Step12, Step23, Step34, Step45=NetworkSrfSettings

        Stations01=np.linspace(SX0, SX1, Step01)
        Stations12=np.linspace(SX1, SX2, Step12)
        Stations23=np.linspace(SX2, SX3, Step23)
        Stations34=np.linspace(SX3, SX4, Step34)
        Stations45=np.linspace(SX4, SX5, Step45)

        # StationRange = np.hstack([Stations01[:-1], Stations12[:-1],
        #                           Stations23[:-1], Stations34[:-1],
        #                           Stations45])
        # Make the part from several lofts and fuse them later
        Surfaces=[]

        for npart, station in enumerate([Stations01[1:], Stations12, Stations23, Stations34, Stations45]):
            C=[]

            for i, XStation in enumerate(station):
                # Create plane normal to x direction
                P=Geom_Plane(gp_Pln(gp_Pnt(XStation, 0, 0),
                                      gp_Dir(gp_Vec(1, 0, 0))))

                IPoint2=act.points_from_intersection(P, FSVUCurve)
                IPoint3=act.points_from_intersection(P, FSVLCurve)
    #
                IPointCentre=act.points_from_intersection(P,
                                                            FSVMeanCurve)

                PseudoDiameter=abs(IPoint3.Z() - IPoint2.Z())

                PseudoRadius=PseudoDiameter / 2.
                # Note: Add Circle with radius PseudoRadius at Pc
                c=GC_MakeCircle(gp_Ax2(IPointCentre, gp_Dir(1, 0, 0)),
                                  PseudoRadius).Value()
                C.append(c)

            if npart == 0:
                # Need to add the leading edge on the first lofted section
                partial_oml=act.AddSurfaceLoft(C, continuity=GeomAbs_C2,
                                                 first_vertex=act.make_vertex(
                                                     self.BowPoint),
                                                 solid=False)
            else:
                partial_oml=act.AddSurfaceLoft(
                    C, continuity=GeomAbs_C2, solid=False)
            Surfaces.append(partial_oml)
            # self['surf' + str(npart)] = partial_oml

        from OCC.BRepBuilderAPI import BRepBuilderAPI_Sewing
        sewing=BRepBuilderAPI_Sewing()
        for surface in Surfaces:
            sewing.Add(surface)
        sewing.Perform()
        self['OML']=sewing.SewedShape()
        # self['OML'] = BRepAlgoAPI_Fuse(*Surfaces)
        return None

    #     """Builds a fuselage shell from primitive shapes

    #     The nose cone will be a truncated ellipsoid, followed by a cylindrical
    #     body, and finally a truncated tail cone.

    #     Notes
    #     -----
    #     Adds the fused shell as a single shape named 'OML' i.e. fuselage['OML']
    #     """
    #     self.BowPoint = gp_Pnt(*self.NoseCoordinates)

    #     kN = self.NoseLengthRatio / 0.182
    #     tN = self.TailLengthRatio / 0.293

    #     NoseEndX = 4 * kN * 2.541
    #     TailStartX = (22 - (22 - 15.55) * tN) * 2.541

    #     NPnt = np.array(self.NoseCoordinates)
    #     # Center of the nose ellipse (full untruncated shape) is the end of the
    #     # Nose section, i.e. at NoseEndX, with no change in y z.
    #     Nose_center = np.array([NoseEndX, NPnt[1], NPnt[2]])

    #     dx, dy, dz = np.subtract(Nose_center - NPnt) * 2
    #     nose = act.make_ellipsoid(Nose_center, dx, dy, dz)

    def BuildFuselageOML(self, Max_attempt=5):
        """Builds the Fuselage outer mould line
        Notes
        -----
        It is not expected that users will interact with this directly. Use
        the Fuslage class initialisation fuction instead
        """
        NetworkSrfSettings=np.array([[35, 20, 15, 15, 20],
                                       [35, 30, 15, 5, 20],
                                       [35, 20, 15, 2, 20],
                                       [30, 30, 15, 2, 20],
                                       [30, 20, 15, 2, 20],
                                       [25, 20, 15, 2, 20],
                                       [20, 20, 15, 2, 20],
                                       [15, 20, 15, 2, 20]])
        HStarboardCurve, HPortCurve, FSVUCurve, FSVLCurve, FSVMeanCurve, \
            NoseEndX, TailStartX, EndX=self.FuselageLongitudinalGuideCurves(self.NoseLengthRatio,
                                                 self.TailLengthRatio)

        # Returning the PortCurve and StarboardCurve as Geom_BSplineCurve
        # makes kernel freeze in pythonocc 0.16.5, so needed to carry around a
        # handle instead - very strange bug!
        PortCurve=HPortCurve .GetObject()
        StarboardCurve=HStarboardCurve.GetObject()

        # Compute the stern point coordinates of the fuselage
        Pu=FSVUCurve.EndPoint()
        Pl=FSVLCurve.EndPoint()
        self.SternPoint=gp_Pnt(Pu.X(), Pu.Y(), 0.5 * (Pu.Z() + Pl.Z()))
        Pu=FSVUCurve.StartPoint()
        Pl=FSVLCurve.StartPoint()
        self.BowPoint=gp_Pnt(Pu.X(), Pu.Y(), 0.5 * (Pu.Z() + Pl.Z()))

        i_attempt=0
        while i_attempt < Max_attempt:
            i_attempt=i_attempt + 1
            log.debug("Surface fit attempt {}".format(i_attempt))

            # Construct array of cross section definition frames
            SX0=0
            SX1=0.04 * NoseEndX
            SX2=SX1 + 0.25 * NoseEndX
            SX3=NoseEndX
            SX4=TailStartX
            SX5=EndX

            Step01, Step12, Step23, Step34, Step45=NetworkSrfSettings[
                i_attempt - 1]

            log.debug("""Attempting loft surface fit with network density
                setup {}""".format(NetworkSrfSettings[i_attempt][:]))
            Stations01=np.linspace(SX0, SX1, max([Step01, 2]))
            Stations12=np.linspace(SX1, SX2, max([Step12, 2]))
            Stations23=np.linspace(SX2, SX3, max([Step23, 2]))
            Stations34=np.linspace(SX3, SX4, max([Step34, 2]))
            Stations45=np.linspace(SX4, SX5, max([Step45, 2]))

            StationRange=np.hstack([Stations01[:-1], Stations12[:-1],
                                      Stations23[:-1], Stations34[:-1],
                                      Stations45])
            C=[]
            FirstTime=True

            for i, XStation in enumerate(StationRange[1:]):
                # Create plane normal to x direction
                P=Geom_Plane(gp_Pln(gp_Pnt(XStation, 0, 0),
                                      gp_Dir(gp_Vec(1, 0, 0))))
                # Make into a face for visualisation/debugging
                try:
                    IPoint2=act.points_from_intersection(P, FSVUCurve)
                    IPoint3=act.points_from_intersection(P, PortCurve)
                    IPoint4=act.points_from_intersection(P, FSVLCurve)
                    IPoint1=act.points_from_intersection(P, StarboardCurve)
#
                    IPointCentre=act.points_from_intersection(P,
                                                                FSVMeanCurve)
                except RuntimeError:
                    log.warning("Intersection Points at Section X={} Not Found"
                                .format(XStation))
                    log.warning("Skipping this plane location")
                    continue

                PseudoDiameter=abs(IPoint4.Z() - IPoint2.Z())
                if self.CylindricalMidSection and\
                        NoseEndX < XStation < TailStartX:
                    log.debug("Enforcing circularity in the central section...")
                    if FirstTime:
                        PseudoRadius=PseudoDiameter / 2.
                        FirstTime=False
                    PseudoRadius=PseudoDiameter / 2.
                    # Note: Add Circle with radius PseudoRadius at Pc
                    from OCC.GC import GC_MakeCircle
                    c=GC_MakeCircle(gp_Ax2(IPointCentre, gp_Dir(1, 0, 0)),
                                      PseudoRadius).Value()

                else:
                    # Set the tangents at each point for interpolation:
                    # assume that these are solely in 1 axis as points lie
                    # extremities of an elliptical shape
                    tangents=np.array([[0, -1, 0],
                                         [0, 0, -1],
                                         [0, 1, 0],
                                         [0, 0, 1]])
                    c=act.points_to_bspline(
                        [IPoint2, IPoint3, IPoint4, IPoint1],
                        periodic=True, scale=False,
                        tangents=tangents)

                C.append(c)

#             Fit fuselage external surface
            sections=[act.make_wire(act.make_edge(curve)) for curve in C]
            guides=([FSVUCurve.GetHandle(), PortCurve.GetHandle(),
                       FSVLCurve.GetHandle(), StarboardCurve.GetHandle()])
            guides=[act.make_wire(act.make_edge(guide)) for guide in guides]
            self._Lguides=guides
            self._Csections=sections
            self._NoseVertex=act.make_vertex(self.BowPoint)
            try:
                OMLSurf=act.AddSurfaceLoft(C, first_vertex=self._NoseVertex,
                                       continuity=GeomAbs_C2, solid=False)
            except:
                OMLSurf=None

            if OMLSurf is not None:
                log.debug("Network surface fit succesful on attempt {}\n"
                          .format(i_attempt))
                self.AddComponent(OMLSurf, 'OML')
                return None

#         If all attempts at fitting a network surface failed, we attempt a
#            Pipe Shell:
        if OMLSurf is None:
            log.warning("""Failed to fit network surface to the external shape of the
                fuselage""")
            log.warning("""Attempting alternative fitting method, quality likely to
                be low...""")

            try:
                OMLSurf=act.make_pipe_shell(C)
                self.AddComponent(OMLSurf, 'OML')
            except:
                raise(RuntimeError, "Could not produce a valid OML surface")

        # Shouldnt get here
        return None

    def TransformOML(self):
        """Use parameters defined in self to scale and translate the fuselage
        """
        ScalingF=np.array(self.Scaling) / 55.902
        # ScalingF[0] = self.Scaling[0] / 55.902
        # ScalingF[1] = self.Scaling[1] / 55.902
        # ScalingF[2] = self.Scaling[2] / 55.902
#
#        # Overall scaling and translation:
        MoveVec=self.NoseCoordinates
        self.TransformComponents_Nonuniformal(ScalingF, MoveVec)

        self.BowPoint=gp_Pnt(*self.NoseCoordinates)

        self.SternPoint=gp_Pnt(self.SternPoint.X(
        ) * ScalingF[0], self.SternPoint.Y(), self.SternPoint.Z() * ScalingF[2])
        self.SternPoint.Translate(gp_Vec(*MoveVec))


#        SternPoint[0] = SternPoint[0]*ScalingF[0]
#        SternPoint[1] = SternPoint[1]*ScalingF[1]
#        SternPoint[2] = SternPoint[2]*ScalingF[2]
#
#        # Positioning
#        SternPoint[0] = SternPoint[0]+NoseCoordinates[0]
#        SternPoint[1] = SternPoint[1]+NoseCoordinates[1]
#        SternPoint[2] = SternPoint[2]+NoseCoordinates[2]

    def CockpitWindowContours(self, Height=1.620, Depth=5):
        """
        This function is currently not tested
        """
        raise NotImplementedError(
            "This function is in development, and its output is untested")
        P1=[0.000, 0.076, Height - 1.620 + 2.194]
        P2=[0.000, 0.852, Height - 1.620 + 2.290]
        P3=[0.000, 0.904, Height + 0.037]
        P4=[0.000, 0.076, Height]
        edge1=act.make_edge(gp_Pnt(*P1), gp_Pnt(*P2))
        edge2=act.make_edge(gp_Pnt(*P2), gp_Pnt(*P3))
        edge3=act.make_edge(gp_Pnt(*P3), gp_Pnt(*P4))
        edge4=act.make_edge(gp_Pnt(*P4), gp_Pnt(*P1))

        wire=act.make_wire([edge1, edge2, edge3, edge4])

        face_inner=act.make_face(wire)  # The inner cockpit window

        filleted_face=act.FilletFaceCorners(face_inner, 0.08)

#        exp = TopExp_Explorer
#        CWC1 =

        # The outer cockpit window
        P1=[0.000, 0.951, Height - 1.620 + 2.289]
        P2=[0.000, 1.343, Height - 1.620 + 2.224]
        P3=[0.000, 1.634, Height - 1.620 + 1.773]
        P4=[0.000, 1.557, Height - 1.620 + 1.588]
        P5=[0.000, 1.027, Height - 1.620 + 1.671]
        edge1=act.make_edge(gp_Pnt(*P1), gp_Pnt(*P2))
        edge2=act.make_edge(gp_Pnt(*P2), gp_Pnt(*P3))
        edge3=act.make_edge(gp_Pnt(*P3), gp_Pnt(*P4))
        edge4=act.make_edge(gp_Pnt(*P4), gp_Pnt(*P5))
        edge5=act.make_edge(gp_Pnt(*P5), gp_Pnt(*P1))

        wire=act.make_wire([edge1, edge2, edge3, edge4, edge5])

        face_outer=act.make_face(wire)  # The inner cockpit window

        CWC2=act.FilletFaceCorners(face_outer, 0.08)

        CWC3=act.mirror(CWC1, plane='xz', copy=True)
        CWC4=act.mirror(CWC2, plane='xz', copy=True)

        return CWC1, CWC2, CWC3, CWC4

    def WindowContour(self, WinCenter):
        """Creates and returns the contour of the window at WinCenter

        Parameters
        ----------
        WinCenter : list or array, length 2
            The [X, Z] coordinate of the center of the window

        Returns
        -------
        W_wire : TopoDS_Wire
            The wire of the B-spline contour
        """
        raise NotImplementedError(
            "This function is in development, and its output is untested")
        P1=gp_Pnt(WinCenter[0], 0, WinCenter[1] + 0.468 / 2.)
        P2=gp_Pnt(WinCenter[0] + 0.272 / 2., 0, WinCenter[1])
        P3=gp_Pnt(WinCenter[0], 0, WinCenter[1] - 0.468 / 2.)
        P4=gp_Pnt(WinCenter[0] - 0.272 / 2., 0, WinCenter[1])

        tangents=np.array([[0, 0, 2.5],
                             [0, 0, -2.5]])

        WCurveU=act.points_to_bspline([P4, P1, P2], tangents=tangents)
        # Need to reverse the tangents for the following shape:
        WCurveL=act.points_to_bspline([P2, P3, P4], tangents=tangents[::-1])

        edgeU=act.make_edge(WCurveU)
        edgeL=act.make_edge(WCurveL)

#        W_face= act.make_face(act.make_wire([edgeU, edgeL]))
        W_wire=act.make_wire([edgeU, edgeL])
        return W_wire

    def MakeWindow(self, Xwc, Zwc):
        """Makes at Window centered at Wxc Zwc using the bspline wire returned
        by WindowContour

        THIS FUNCTION IS IN DEVELOPMENT AND NOT YET TESTED FULLY

        Parameters
        ----------
        Xwc : scalar
            The window center x coordinate

        Zwc : scalar
            The window center z coordinate

        Returns
        -------
        WinStbd : TopoDS_Shape
            The window surface cut out (starboard side)

        WinPort : TopoDS_Shape
            The window surface cut out (Port side)

        Notes
        -----
        Changes the contents of self['OML'].
        Makes both the port and starboard windows at the input location.
        """
        raise NotImplementedError(
            "This function is in development, and its output is untested")
        WinCenter=[Xwc, Zwc]
        W_wire=self.WindowContour(WinCenter)

        ExtPathStbd=gp_Dir(gp_Vec(gp_Pnt(0, 0, 0), gp_Pnt(0, 10, 0)))
        ExtPathPort=gp_Dir(gp_Vec(0, -10, 0))

        self['OML'], WinStbd=act.SplitShapeFromProjection(self['OML'], W_wire,
                                                            direction=ExtPathStbd)

        self['OML'], WinPort=act.SplitShapeFromProjection(self['OML'], W_wire,
                                                            direction=ExtPathPort)

        return WinStbd, WinPort

    def fit_scale_location(self, oldscaling, oldx, oldy, oldz):
        """Given an old scaling and old position (this will be a random number
        between 0 and 1 using the DEAP tree I have set up elsewhere), a new
        scaling and position is returned allowing the resulting shape to 'fit'
        to its parent"""
        # Need to get a scaling from the parent of arbitrary type:
        # using a try-except to work for any parent type ... could probably
        # do better here

        scalingrange = np.array([0.1, 1])

        parentscalefactor = self.Scaling[0]

        parent_apex = self.BowPoint
        # dL = gp_Vec(parent.BowPoint, parent.SternPoint)
        # self._testpoints.append(parent.SternPoint)
        xmin, ymin, zmin, xmax, ymax, zmax = self.Extents()

        newx = parent_apex.X() + (xmax - xmin) * oldx
        newy = parent_apex.Y() + (ymax - ymin) / 2. * oldy
        newz = parent_apex.Z() + (zmax - zmin) / 2. * oldz


        # Ensure that the oldscaled and oldposition fractions doesn't give
        # something invisibly small:
        # REMOVING THIS TEMPORARILY: trying to see if scalings > 1 are allowed
        # oldscaling = np.interp(oldscaling, [0, 1], scalingrange)

        # The scaling is some percentage of parent (assumes components get
        # smaller)
        newscaling = oldscaling * parentscalefactor


        return newscaling, newx, newy, newz

#    def MakeCockpitWindows(self, Height = 1.620, Depth = 5):
#        CW
###############################################################################

    def get_spanstation_chord(self, SpanStation):
        """
        """
        EngineSection, HChord = act.CutSect(self['OML'], SpanStation)
        return HChord
