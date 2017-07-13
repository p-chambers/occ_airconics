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
from six.moves import range
import numpy as np
from .base import AirconicsShape
from .primitives import Airfoil
from . import AirCONICStools as act

from OCC.gp import gp_Pnt, gp_Vec, gp_XOY, gp_Ax3, gp_Dir
from OCC.GeomAbs import GeomAbs_C2
from OCC.Geom import Geom_Plane

import logging
log = logging.getLogger(__name__)

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

    def uniform_parametric_function(epsilon):
        """Does nothing with spanwise parameter epsilon, but is required for
        the general definition of shape in the lifting surface class"""
        return value
    return uniform_parametric_function


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

    Attributes
    ----------
    self['Surface'] : TopoDS_Shape
        The generated lifting surface

    Sections : list of airconics.primitives.Airfoil objects
        The rib curves from which the main surface is lofted. Updating any of
        the spanwise functions (ChordFunct, TwistFunct, etc...), SpanFactor,
        ChordFactor Raises an
        error if attempting to write over it manually.

    RootChord : Scalar
        The length of the Root Chord. Updated by GenerateLiftingSurface

    AR : Scalar
        The Aspect ratio of the Lifting Surface. Updated on call to Build

    LSP_area : Scalar
        the projected area of the lifting surface. Updated on call to Build

    SA : Scalar
        The wetted area of the lifting surface. Updated on call to Build

    ActualSemiSpan : Scalar
        Calculated semi span of the lifting surface. Updated on call to Build

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
                 ):
        # convert ApexPoint from list if necessary
        try:
            ApexPoint = gp_Pnt(*ApexPoint)
        except:
            pass

        if not (SweepFunct or DihedralFunct or TwistFunct or ChordFunct or
                AirfoilFunct):
            log.debug("Lifting Surface functional parameters not defined:")
            log.debug("Initialising without geometry construction")
            construct_geometry = False

        self.CreateConstructionGeometry()

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
                                             LSP_area=None,
                                             AR=None,
                                             ActualSemiSpan=None,
                                             RootChord=None,
                                             SA=None,
                                             OptimizeChordScale=OptimizeChordScale,
                                             LooseSurf=LooseSurf,
                                             _NSegments=SegmentNo,
                                             TipRequired=TipRequired,
                                             max_degree=max_degree,
                                             Cont=continuity,
                                             construct_geometry=construct_geometry
                                             )

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

    def CreateConstructionGeometry(self):
        """
        Creates the plane and vector used for projecting wetted area
        """
        self.XoY_Plane = Geom_Plane(gp_Ax3(gp_XOY()))
        self.ProjVectorZ = gp_Dir(0, 0, 1)

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

        # Also store the tip leading edge point (for fitting tip devices)?
        # self.TipLE = self.Sections[-1].Chord.

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
        LEPoints = np.zeros((self.NSegments + 1, 3))

        Deltas = np.vstack([DeltaXs, DeltaYs, DeltaZs]).T
        LEPoints[1:, :] = np.cumsum(Deltas, axis=0)

        self.LEPoints = LEPoints

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

        self.LECurve = act.points_to_bspline(LEPoints)

        Eps = np.linspace(0, 1, self.NSegments + 1)

        for i, eps in enumerate(Eps):
            Af = self.AirfoilFunct(Epsilon=eps,
                                   LEPoint=LEPoints[i],
                                   ChordFunct=self.ChordFunct,
                                   ChordFactor=self.ChordFactor,
                                   DihedralFunct=self.DihedralFunct,
                                   TwistFunct=self.TwistFunct)
            self._Sections.append(Af)

    # def ChordScaleOptimizer(self):
    #     """
    #     """
    #     raise NotImplementedError
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
                                continuity=self.Cont,
                                solid=False)

        # TODO: Optimize chord scale ...
        # if self.OptimizeChordScale:
            # self.ChordScaleOptimizer()

        if LS is None:
            raise ValueError("Failed to generate Lofted Surface")
        #######################################################################
            # TODO: backup surface loft is not yet implemented for OCC
            # Version of Airconics (this is legacy from the Rhino plugin)
        #######################################################################

        #  Update instance components:
        self.AddComponent(LS, 'Surface')

        # Calculate projected area
        self.LSP_area = self.CalculateProjectedArea()

        if self.TipRequired:
            # TODO: retrieve wing tip
            log.warning("Tip Required currently does nothing")
            WingTip = None
            # self["Tip"] = WingTip

        # Scaling (w.r.t. origin)
        if self.ScaleFactor != 1:
            self.ScaleComponents_Uniformal(self.ScaleFactor)
            self.LECurve.GetObject().Scale(gp_Pnt(0,0,0), self.ScaleFactor)

        # Position the Components at the apex:
        vec = gp_Vec(gp_Pnt(0., 0., 0.), self.ApexPoint)
        self.TranslateComponents(vec)
        self.LECurve.GetObject().Translate(vec)

        # Calculate some parameters
        self.RootChord = (self.ChordFunct(0) * self.ChordFactor *
                          self.ScaleFactor)
        self.ActualSemiSpan = self.CalculateSemiSpan()
        self.AR = self.CalculateAspectRatio()
        self.SA = act.CalculateSurfaceArea(self['Surface'])

        log.info("""Lifting Surface complete. Key features:
    Proj.area: {},
    Wet.area: {},
    Span:{},
    Aspect ratio: {},
    Root chord: {}\n""".format(self.LSP_area, self.SA, self.ActualSemiSpan,
                               self.AR, self.RootChord))

        return None

    def CalculateProjectedArea(self):
        """Calculates the projected area of the current lifting surface

        From Airconics documentation: In some cases the projected section
        cannot all be lofted in one go (it happens when parts of the wing fold
        back onto themselves), so we loft them section by section and compute
        the area as a sum.
        """
        assert(self['Surface']), 'No wing surface found. Try running Build.'

        # First project the section chords onto the xy plane:
        ProjectedSections = []
        for section in self.Sections:
            ProjectedSections.append(
                act.project_curve_to_plane(section.ChordLine,
                                           self.XoY_Plane,
                                           self.ProjVectorZ))

        try:
            # Loft a section for each projected segment and get its area
            LSP_area = 0
            for i in range(self.NSegments):

                LSPsegment = act.AddSurfaceLoft(ProjectedSections[i: i + 2],
                                                close_sections=False)
                SA = act.CalculateSurfaceArea(LSPsegment)
                LSP_area += SA
        except:
            log.warning("""Failed to compute projected area. Using half of surface
                area instead.""")
            LS_area = act.CalculateSurfaceArea(self['Surface'])
            LSP_area = 0.5 * LS_area

        # Scale the area
        LSP_area *= self.ScaleFactor ** 2.0

        return LSP_area

    def CalculateSemiSpan(self):
        """Calculates and returns the span of this lifting surface.

        Uses the OCC bounding box algorithm.

        Returns
        -------
        ActualSemiSpan : Scalar
        """
        # Check bounding box size
        BB = self.Extents(as_vec=True)
        if BB:
            ActualSemiSpan = BB[1].Y() - BB[0].Y()   # max y - min y
        else:
            ActualSemiSpan = 0.0
        return ActualSemiSpan

    def CalculateAspectRatio(self):
        """Calculates and returns the aspect ratio of this lifting surface

        Uses information about the wings projected area and the current
        bounding box. If the project area (LSP_Area) is zero, this will be
        calculated.

        Returns
        -------
        AR : Scalar
            Aspect ratio, calculated by b^2 / A, where b is the semi span and
            A is the project area of this lifting surface

        See Also
        --------
        airconics.LiftingSurface.ProjectedArea,
        airconics.LiftingSurface.SemiSpan
        """
        if not self.LSP_area:
            LSP_area = self.CalculateProjectedArea()
        else:
            LSP_area = self.LSP_area

        if not self.ActualSemiSpan:
            ActualSemiSpan = self.CalculateSemiSpan()
        else:
            ActualSemiSpan = self.ActualSemiSpan
        AR = ((ActualSemiSpan) ** 2.0) / (LSP_area)
        return AR

    def Fit_BlendedTipDevice(self, rootchord_norm, spanfraction=0.1, cant=40,
                             transition=0.1, sweep=40, taper=0.7):
        """Fits a blended wing tip device [1],

        Parameters
        ----------
        rootchord_norm : scalar
            The root chord of the straight part of the winglet, normalised by
            the tip chord of this lifting surface.

        spanfraction : scalar
            span of the winglet normalised by the span of the main wing

        cant : scalar (default 40)
            Angle (deg) of the wing tip from vertical

        transition : scalar
            The percentage along the span at which the transition to a straight
            segment is located
        
        sweep : scalar
            Sweep angle of the wing tip (uniform along the straight section)

        taper : scalar
            ratio of the tip chord to the root chord of the straight segment

         References
        ----------
        [1] L. B. Gratzer, "Blended winglet," Google Patents, 1994
        """
        # The array of spanwise locations at which values of sweep, chord etc
        # are known
        Eps_Array = np.array([0, transition, 1])
        Dihedrals = np.array([self.DihedralFunct(1),
                              90 - cant,
                              90 - cant])
        Sweeps = np.array([self.SweepFunct(1), sweep, sweep])
        # No change in washout along the winglet
        Twists = np.ones(3) * self.TwistFunct(1)
        # Normalised chord (will be scaled st root chord = main wing tip chord)
        Chords = np.array([1, rootchord_norm, taper * rootchord_norm])

        # Generating all the interpolated transition functions
        DihedralFunctWinglet = act.Generate_InterpFunction(Dihedrals,
                                                           Eps_Array)

        # if linear_TE:
        #     # If a linear trailing edge is selected, offset the LE from

        # else:
        SweepFunctWinglet = act.Generate_InterpFunction(Sweeps,
                                                        Eps_Array)
        ChordFunctWinglet = act.Generate_InterpFunction(Chords,
                                                        Eps_Array)
        TwistFunctWinglet = act.Generate_InterpFunction(Twists,
                                                        Eps_Array)

        # No change in airfoil profile
        @airfoilfunct
        def AirfoilFunctWinglet(Epsilon):
            return self.Sections[-1].Profile

        apex_array = np.array([self.ApexPoint.X(), self.ApexPoint.Y(),
            self.ApexPoint.Z()])
        Winglet_apex = apex_array + np.array(self.LEPoints[-1]) * self.ScaleFactor

        # Scale the entire wing to get the span as a percentage of this wing
        scalefactor = self.ScaleFactor * spanfraction

        # Scale the winglet root chord to match the tip chord of this wing
        chordfactor = ((self.ChordFunct(1) * self.ChordFactor) /
                       (ChordFunctWinglet(0) * spanfraction)
                       )

        Winglet = LiftingSurface(ApexPoint=Winglet_apex,
                                 SweepFunct=SweepFunctWinglet,
                                 DihedralFunct=DihedralFunctWinglet,
                                 TwistFunct=TwistFunctWinglet,
                                 ChordFunct=ChordFunctWinglet,
                                 AirfoilFunct=AirfoilFunctWinglet,
                                 ScaleFactor=scalefactor,
                                 ChordFactor=chordfactor)

        return Winglet

    def FitScaleLocation(self, XScaleFactor, oldx, oldy, oldz, base_xlength):
        """Given an old scaling and old position (this will be a random number
        between 0 and 1 using the DEAP tree I have set up elsewhere), a new
        scaling and position is returned allowing the resulting shape to 'fit'
        to its parent
        """
        # The interpolation along the curve: this could be done better
        # Essentially, this is the length of the vector normalised by
        # the diagonal of a 1 by 1 by 1 cube
        curve = self.LECurve.GetObject()
        epsilon = np.linalg.norm([oldx, oldy, oldz]) / np.sqrt(3)
        newapex = curve.Value(epsilon)
        newx, newy, newz = newapex.X(), newapex.Y(), newapex.Z()

        # Ensure that the oldscaled and oldposition fractions does give
        # something invisibly small:
        # REMOVING THIS TEMPORARILY: trying to see if scalings > 1 are allowed
        # oldscaling = np.interp(oldscaling, [0, 1], scalingrange)

        # The scaling is some percentage of parent (assumes components get
        # smaller)
        Chord = self.get_spanstation_chord(epsilon).GetObject()
        fitting_length = abs(Chord.StartPoint().X() - Chord.EndPoint().X()) * XScaleFactor
        newscaling = fitting_length / float(base_xlength)

        return newscaling, newx, newy, newz

    def get_spanstation_chord(self, SpanStation):
        """
        """
        Section, HChord = act.CutSect(self['Surface'], SpanStation)
        return HChord
