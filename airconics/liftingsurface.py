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
from airconics.base import AirconicsShape
from airconics.primitives import Airfoil
import AirCONICStools as act

from OCC.gp import gp_Pnt, gp_Vec
from OCC.GeomAbs import GeomAbs_C2


def airfoilfunct(ProfileFunct):
    """Decorator function : creates and returns a functional parameter
    AirfoilFunct(epsilon), where epsilon is the spanwise coordinate, given the
    input profile function

    Parameters
    ----------
    ProfileFunct : function
        This function should return a dictionary of KEYWORD: VALUE pairs,
        for all keywords required to define to the airfoil profile at
        spanwise location epsilon, e.g.,

        >>> def my_simple_profilfunct(eps):
        >>>     return {'NACA4Profile': '0012'}

    Returns
    -------
    AirfoilFunct : function
        A wrapper to the input ProfileFunct. Takes in Epsilon, LEPoint,
        ChordFunct, ChordFactor, DihedralFunct, TwistFunct as inputs, and
        passes all arguments (and information about the profile from
        ProfileFunct), to the airconics.Airfoil class.
        - Returns the generated airfoil.

    Notes
    -----
    Example:

    >>> @airfoilfunct
    >>> def my_AirfoilFunct(eps)
    >>>     return {'NACA4Profile': '0012'}
    >>> Wing = LiftingSurface((0,0,0), mySweepFunct,
                              myDihedralFunct,
                              myTwistFunct,
                              myChordFunct,
                              my_AirfoilFunct,
                              )

    Where mySweepFunct has been previously defined.

    See Also
    --------
    airconics.primitives.Airfoil,
    core examples liftingsurface_airfoilfunct_decorator
    """
    def AirfoilFunct(Epsilon, LEPoint, ChordFunct, ChordFactor,
                     DihedralFunct, TwistFunct):
        # Wraps ProfileFunct
        Profile_Dict = ProfileFunct(Epsilon)

        AfChord = ((ChordFactor * ChordFunct(Epsilon)) /
                   np.cos(np.radians(TwistFunct(Epsilon))))

        Af = Airfoil(LEPoint, ChordLength=AfChord,
                     Rotation=DihedralFunct(Epsilon),
                     Twist=TwistFunct(Epsilon),
                     **Profile_Dict)
        return Af
    return AirfoilFunct


class LiftingSurface(AirconicsShape):
    """Airconics class for defining lifting surface shapes

    Parameters
    ----------
    ApexPoint - array, length 3
        Foremost point of the wing (x direction). Updating will rebuild the
        geometry.

    SweepFunct - function
        function defining the leading edge sweep vs epsilon spanwise
        variable coordinate between 0 and 1 (curvilinear attached
        coordinates). Updating will rebuild the geometry.

    DihedralFunct - function
        function defining the leading edge dihedral vs epsilon spanwise
        variable coordinate between 0 and 1 (curvilinear attached).
        Updating will rebuild the geometry.

    TwistFunc - function
        function defining the sectional twist vs epsilon spanwise
        variable coordinate between 0 and 1 (curvilinear attached).
        Updating will rebuild the geometry.

    ChordFunct - function
        function defining the leading edge chord vs epsilon spanwise
        variable coordinate between 0 and 1 (curvilinear attached)
        Updating will rebuild the geometry.

    AirfoilFunct - function
        function defining the sectional Airfoil (see primitives.Airfoil)
        vs epsilon spanwise variable coordinate between 0 and 1
        (curvilinear attached).
        Updating will rebuild the geometry.

    ChordFactor - int (default = 1)
        Scaling factor applied in chordwise direction. Updating will rebuild
        the geometry.

    ScaleFactor - int (default = 1)
        Scaling factor applied in all directions (uniform). Updating will
        rebuild the geometry.

    OptimizeChordScale - int or bool (default = 0)
        TODO: Not yet used.

    LooseSurf - (default = 1)
        TODO:

    NSegments - int (default = 11)
        Number of segments to sample the wing defined by input functions.
        Updating will rebuild the geometry.

    TipRequired - bool (default = False)
        TODO: Not yet used
        adds the wing tip face to components if true

    max_degree - (default = 8)
        maximum degree of the fitted NURBS surface

    continuity - OCC.GeomAbs.GeomAbs_XX Type
        the order of continuity i.e. C^0, C^1, C^2... would be
        GeomAbs_C0, GeomAbs_C1, GeomAbs_C2 ...

    construct_geometry : bool
        If true, Build method will be called on construction

    randomize : bool
        If true, a random set of inputs will be selected. See also:
        airconics.LiftingSurface.Randomize

    Attributes
    ----------
    self['Surface'] : TopoDS_Shape
        The generated lifting surface

    Sections : list of OCC Handle_GeomBSplineCurve objects
        The rib curves from which the main surface is lofted. Updating any of
        the spanwise functions (ChordFunct, TwistFunct, etc...), SpanFactor,
        ChordFactor Raises an
        error if attempting to write over it manually.

    RootChord : Scalar
        The length of the Root Chord. Updated by GenerateLiftingSurface


    Notes
    -----
    * Output surface is stored in self['Surface']
    *
    * See airconics.examples.wing_example_transonic_airliner for
      example input functions

    See also
    --------
    airconics.primitives.Airfoil,
    airconics.examples.wing_example_transonic_airliner

    """

    # Preset lifting surface options:

    PRESETS = {'SweepFunct': None}   # TODO: populate this

    def __init__(self, ApexPoint=gp_Pnt(0, 0, 0),
                 SweepFunct=False,
                 DihedralFunct=False,
                 TwistFunct=False,
                 ChordFunct=False,
                 AirfoilFunct=False,
                 ChordFactor=1,
                 ScaleFactor=1,
                 OptimizeChordScale=0,
                 LooseSurf=1,
                 SegmentNo=11,
                 TipRequired=False,
                 max_degree=8,
                 continuity=GeomAbs_C2,
                 construct_geometry=True,
                 randomize=False,
                 parent=None
                 ):
        # convert ApexPoint from list if necessary
        try:
            ApexPoint = gp_Pnt(*ApexPoint)
        except:
            pass

        if not (SweepFunct or DihedralFunct or TwistFunct or ChordFunct or
                AirfoilFunct):
            print("Lifting Surface functional parameters not defined:")
            print("Initialising without geometry construction")
            construct_geometry = False

#        Initialise the components using base class:
        super(LiftingSurface, self).__init__(components={},
                                             _ApexPoint=ApexPoint,
                                             _SweepFunct=SweepFunct,
                                             _DihedralFunct=DihedralFunct,
                                             _TwistFunct=TwistFunct,
                                             _ChordFunct=ChordFunct,
                                             _AirfoilFunct=AirfoilFunct,
                                             _ChordFactor=ChordFactor,
                                             _ScaleFactor=ScaleFactor,
                                             _Sections=[],
                                             RootChord=None,
                                             OptimizeChordScale=OptimizeChordScale,
                                             LooseSurf=LooseSurf,
                                             _NSegments=SegmentNo,
                                             TipRequired=TipRequired,
                                             max_degree=max_degree,
                                             Cont=continuity,
                                             construct_geometry=construct_geometry,
                                             randomize=randomize,
                                             parent=parent)

#        self.CreateConstructionGeometry()

    # Properties
    # ----------------------------------------------------------------------
    # Apex Point
    @property
    def ApexPoint(self):
        return self._ApexPoint

    @ApexPoint.setter
    def ApexPoint(self, newApexPoint):
        vec = gp_Vec(self.ApexPoint, newApexPoint)
        self._ApexPoint = newApexPoint
        self.TranslateComponents(vec)

    # Sweep
    @property
    def SweepFunct(self):
        return self._SweepFunct

    @SweepFunct.setter
    def SweepFunct(self, newSweepFunct):
        # Maybe add some tests here
        self._SweepFunct = newSweepFunct
        # Only rebuild if Build was previously successful (if some components
        # have been created in self)
        if self.construct_geometry:
            self.Build()

    # Dihedral
    @property
    def DihedralFunct(self):
        return self._DihedralFunct

    @DihedralFunct.setter
    def DihedralFunct(self, newDihedralFunct):
        # Maybe add some tests here
        self._DihedralFunct = newDihedralFunct
        if self.construct_geometry:
            self.Build()

    # Twist
    @property
    def TwistFunct(self):
        return self._TwistFunct

    @TwistFunct.setter
    def TwistFunct(self, newTwistFunct):
        # Maybe add some tests here
        self._TwistFunct = newTwistFunct
        if self.construct_geometry:
            self.Build()

    # Chord
    @property
    def ChordFunct(self):
        return self._ChordFunct

    @ChordFunct.setter
    def ChordFunct(self, newChordFunct):
        # Maybe add some tests here
        self._ChordFunct = newChordFunct
        if self.construct_geometry:
            self.Build()

    # Airfoil
    @property
    def AirfoilFunct(self):
        return self._AirfoilFunct

    @AirfoilFunct.setter
    def AirfoilFunct(self, newAirfoilFunct):
        # Maybe add some tests here
        self._AirfoilFunct = newAirfoilFunct
        if self.construct_geometry:
            self.Build()

    @property
    def NSegments(self):
        return self._NSegments

    @NSegments.setter
    def NSegments(self, newNSegments):
        self._NSegments = newNSegments
        if self.construct_geometry:
            self.Build()

    @property
    def ChordFactor(self):
        return self._ChordFactor

    @ChordFactor.setter
    def ChordFactor(self, newChordFactor):
        self._ChordFactor = newChordFactor
        # Note: may eventually try to scale rather than rebuild here for speed
        if self.construct_geometry:
            self.Build()

    @property
    def ScaleFactor(self):
        return self._ScaleFactor

    @ScaleFactor.setter
    def ScaleFactor(self, newScaleFactor):
        self._ScaleFactor = newScaleFactor
        # No need to rebuild surface here, just scale w.r.t Apex
        self.ScaleComponents_Uniformal(self._ScaleFactor, self.ApexPoint)

    @property
    def Sections(self):
        # Note: there is no setter for this, as it should not be directly
        # interacted with
        return self._Sections
    # ----------------------------------------------------------------------

    # def SetAirfoilFunct_from_list(self, Profile_list):
    #     """Creates and returns a functional parameter AirfoilFunct(epsilon),
    #     where epsilon is the spanwise coordinate, given the input list of
    #     [(AirfoilProfileType, AirfoilProfile, Epsilon), (...), ....]

    #     Parameters
    #     ----------
    #     Profile_list - list of tuples
    #         Each tuple in Profile_list should be of the form:
    #             >>> (AirfoilProfileType, AirfoilProfile, Epsilon)
    #         where AirfoilProfileType is one of the defined Profiles in
    #         airconics.primitives.Airfoil, e.g., 'Naca4Profile', 'SeligProfile',
    #         AirfoilProfile is the selected Profile of that type, and Epsilon is
    #         the spanwise coordinate (0 at root, 1 at tip) at which that airfoil
    #         exists.

    #     Notes
    #     -----
    #     Example:

    #     >>> af = Airfoil()
    #     >>> af.SetAirfoilFunct_from_list()

    #     See Also
    #     --------
    #     airconics.primitives.Airfoil
    #     """
    #     for profile in Profile_list:

    def Build(self):
        """Builds the section curves and lifting surface using the current

        Uses the current ChordFactor, ScaleFactor, NSegments, ApexPoint, and
        all spanwise variation functions (e.g. ChordFunct) defined in self to
        produce a surface

        Notes
        -----
        Called on initialisation of a lifting surface class. 

        :Example:
            >>> Wing = liftingsurface.LiftingSurface(P,
                                                mySweepAngleFunction,
                                                myDihedralFunction,
                                                myTwistFunction,
                                                myChordFunction,
                                                myAirfoilFunction)
            >>> Surface = Wing['Surface']

        See Also
        --------
        airconics.examples.wing_example_transonic_airliner
        """
        super(LiftingSurface, self).Build()
        assert(self.SweepFunct), 'No sweep function defined'
        assert(self.ChordFunct), 'No Chord function defined'
        assert(self.AirfoilFunct), 'No Airfoil function defined'
        assert(self.TwistFunct), 'No Twist function defined'
        assert(self.DihedralFunct), 'No Dihedral function defined'

        # The first time all functions are successfully defined, change the
        # build flag to true so that further changes to these functions will
        # rebuild geometry via the setter property functions
        self.construct_geometry = True

        self.GenerateSectionCurves()
        self.GenerateLiftingSurface()

    def Randomize(self, parent=None):
        """Randomizes the input parameters defining this LiftingSurface object,
        either free standing or with some reference to a 'parent' node to which
        it is attached.

        Parameters
        ----------
        parent : Fuselage, LiftingSurface or Engine (default : None)
          The parent shape to which this randomised fuselage will be fitted. If
          None, the ApexPoint will be (0,0,0), otherwise the ApexPoint will be
          placed according to the type and shape of the geometry of parent.

        Notes
        -----
        This method can be called on initialisation of the class using the flag
        'randomize' as True, e.g.

        >>> random_wing = LiftingSurface(randomize=True)

        See Also
        --------
        airconics.topology.Topology
        """
        if parent:
            raise NotImplementedError(
                "Randomize method does not yet work with parent geometry node")
        else:
            self.ApexPoint = gp_Pnt(0., 0., 0.)
            self.SweepFunct=False
            self.DihedralFunct=False
            self.TwistFunct=False
            self.ChordFunct=False
            self.AirfoilFunct=False

    def GenerateLeadingEdge(self):
        """Epsilon coordinate attached to leading edge defines sweep
         Returns airfoil leading edge points
         """
        SegmentLength = 1.0 / self.NSegments

#       Array of epsilon at segment midpoints (will evaluate curve here)
        Epsilon_midpoints = np.linspace(SegmentLength / 2.,
                                        1 - (SegmentLength / 2.),
                                        self.NSegments)

#       We are essentially reconstructing a curve from known slopes at
#       known curve length stations - a sort of Hermite interpolation
#       without knowing the ordinate values. If NSegments -> Inf, the
#       actual slope at each point -> the sweep angle specified by
#       SweepFunct
        Tilt_array = self.DihedralFunct(Epsilon_midpoints)
        Sweep_array = self.SweepFunct(Epsilon_midpoints)

        # If length of Tilt array and epsilon midpoints does not match, then
        # the input function is not numpy compatible: use a for loop instead
        if np.shape(Tilt_array) != Epsilon_midpoints.shape:
            Tilt_array = np.zeros_like(Epsilon_midpoints)
            Sweep_array = np.zeros_like(Epsilon_midpoints)
            for i, eps_mid in enumerate(Epsilon_midpoints):
                Tilt_array[i] = self.DihedralFunct(eps_mid)
                Sweep_array[i] = self.SweepFunct(eps_mid)

        DeltaXs = SegmentLength * np.sin(Sweep_array * (np.pi / 180.))
        DeltaYs = SegmentLength * np.cos(Tilt_array * np.pi / 180.) * \
            np.cos(Sweep_array * np.pi / 180.)
        DeltaZs = DeltaYs * np.tan(Tilt_array * np.pi / 180.)

#        Initialise LE coordinate arrays and add first OCC gp_pnt at [0,0,0]:
#        Note: Might be faster to bypass XLE arrays and use local x only
#        XLE = np.zeros(self.NSegments + 1)
#        YLE = np.zeros(self.NSegments + 1)
#        ZLE = np.zeros(self.NSegments + 1)
#        LEPoints = [gp_Pnt(XLE[0], YLE[0], ZLE[0])]
        LEPoints = np.zeros((self.NSegments + 1, 3))

#        for i in xrange(self.NSegments):
#            XLE[i+1] = XLE[i] + DeltaXs[i]
#            YLE[i+1] = YLE[i] + DeltaYs[i]
#            ZLE[i+1] = ZLE[i] + DeltaZs[i]
#            LEPoints[i+1, :] = XLE[i+1], YLE[i+1], ZLE[i+1]
        Deltas = np.vstack([DeltaXs, DeltaYs, DeltaZs]).T
        LEPoints[1:, :] = np.cumsum(Deltas, axis=0)

        return LEPoints

    def GenerateSectionCurves(self):
        """Generates the loft section curves  based on the current
        functional parameters and ChordFactor of the object. 

        Uses self._AirfoilFunct, _ChordFunct etc. and other attributes to
        update the content of self._Sections.

        Returns
        -------
        None
        """
        # Empty the current geometry
        self._Sections = []

        LEPoints = self.GenerateLeadingEdge()

        # TODO: These lists are used for when the curve has been smoothed or
        # the loft has failed, neither of which have been implemented yet
#        ProjectedSections = []
#        TEPoints_u = []
#        TEPoints_l = []

        Eps = np.linspace(0, 1, self.NSegments + 1)

        for i, eps in enumerate(Eps):
                self._Sections.append(self.AirfoilFunct(eps,
                                                        LEPoints[i],
                                                        self.ChordFunct,
                                                        self.ChordFactor,
                                                        self.DihedralFunct,
                                                        self.TwistFunct).Curve)

    def ChordScaleOptimizer(self):
        """
        """
        raise NotImplementedError
        # TODO
#                self._CheckOptParCorrectlySpec()
#                self._NormaliseWeightings()
#                self._PrintTargetsAndWeights()
#                print("Optimizing scale factors...")
#                # An iterative local hillclimber type optimizer is needed
#                    here. One option might be SciPy's fmin as below:
#                # x0, fopt, iter, funcalls, warnflag, allvecs =
#                    scipy.optimize.fmin(self._LSObjective, x0, retall=True,
#                                        xtol=0.025, full_output=True)
#                # However, SciPy is not supported on 64-bit Rhino
#                    installations, so
#                # so here use an alternative: a simple evoltionary optimizer
#                # included with AirCONICS_tools.
#                MaxIter = 50
#                xtol = 0.025
#                deltax = [x0[0]*0.25,x0[1]*0.25]
#                x0, fopt = act.boxevopmin2d(self._LSObjective, x0, deltax,
#                                            xtol, MaxIter)
#                x0[0] = abs(x0[0])
#                x0[1] = abs(x0[1])
#                print("Optimum chord factor %5.3f, optimum scale factor %5.3f"
#                                                       % (x0[0], x0[1]))

    def GenerateLiftingSurface(self):
        """Builds a lifting surface (wing, tailplane, etc.) with the Chord and
        Scale factors, and section list defined in self.

        This function should be called after GenerateSectionCurves. Note that
        both operations are performed with `Build', which should be used
        preferentially

        Returns
        -------
        None

        Notes
        -----
        Adds a ('Surface': Shape) key value pair to self.
        """
        x0 = [self.ChordFactor, self.ScaleFactor]

        LS = act.AddSurfaceLoft(self._Sections,
                                max_degree=self.max_degree,
                                continuity=self.Cont)

        # TODO: Optimize chord scale ...
        if self.OptimizeChordScale:
            self.ChordScaleOptimizer()

        if LS is None:
            raise ValueError("Failed to generate Lofted Surface")
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
#            # AddSweep fails if they are, so if that is the case, one is
#                skipped
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

        #  Update instance components:
        self.AddComponent(LS, 'Surface')

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
        #            print "Failed to compute projected area. Using half of surface
        #                    area instead."
        #            LS_area = rs.SurfaceArea(LS)
        #            LSP_area = 0.5*LS_area[0]

        # TODO: Check bounding box size
        #        BB = rs.BoundingBox(LS)
        #        if BB:
        #            ActualSemiSpan = BB[2].Y - BB[0].Y
        #        else:
        #            ActualSemiSpan = 0.0

        if self.TipRequired:
            # TODO: retrieve wing tip
            print("Warning: Tip Required currently does nothing")
            WingTip = None
            # self["Tip"] = WingTip

        # Scaling (w.r.t. origin)
        if self.ScaleFactor != 1:
            self.ScaleComponents_Uniformal(self.ScaleFactor)

        # TODO: Calculate key features
        # ActualSemiSpan = ActualSemiSpan * ScaleFactor
        # LSP_area = LSP_area * ScaleFactor ** 2.0
        RootChord = (self.ChordFunct(0) * self.ChordFactor) * self.ScaleFactor
        # AR = ((2.0 * ActualSemiSpan) ** 2.0) / (2 * LSP_area)

        self.RootChord = RootChord

        # Position the Components at the apex:
        vec = gp_Vec(gp_Pnt(0., 0., 0.), self.ApexPoint)
        self.TranslateComponents(vec)

        print("Lifting Surface complete. Key features (both wings):")
        # print("Proj.area: %5.4f   Wet.area: %5.4f   Span:%5.4f  Aspect ratio:  %5.4f  Root chord: %5.4f" % (2*LSP_area, 2.0*SA[0], 2.0*ActualSemiSpan, AR, RootChord))
        print("Root Chord: {}. Remaining params to be implemented".format(
            RootChord))

        return None

# TODO: CreateConstructionGeometry from rhino plugin needs migrating to OCC?
#    def CreateConstructionGeometry(self):
#        self.PCG1 = rs.AddPoint([-100,-100,0])
#        self.PCG2 = rs.AddPoint([-100,100,0])
#        self.PCG3 = rs.AddPoint([100,100,0])
#        self.PCG4 = rs.AddPoint([100,-100,0])
#        self.XoY_Plane = rs.AddSrfPt([self.PCG1, self.PCG2, self.PCG3,
#                                      self.PCG4])
#        self.PCG5 = rs.AddPoint([3,3,0])
#        self.PCG6 = rs.AddPoint([3,3,100])
#        self.ProjVectorZ = rs.VectorCreate(self.PCG5, self.PCG6)
