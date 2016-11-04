# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 16:30:02 2015

@author: pchambers
"""

__all__ = ['base', 'primitives', 'AirCONICStools', 'liftingsurface',
           'fuselage_oml', 'engine', 'topology']

import pkg_resources
__version__ = pkg_resources.require("airconics")[0].version

from . import base
from . import primitives
from . import AirCONICStools
from . import liftingsurface
from . import examples
from . import fuselage_oml
from . import engine
from . import topology
#from . import aircraft

# Also allow module level imports for the primary classes (neater API)
from .fuselage_oml import Fuselage
from .liftingsurface import LiftingSurface
from .topology import Topology
from .engine import Engine
from .primitives import Airfoil


# Set default logging handler to avoid "No handler found" warnings.
import sys
import logging
try:  # Python 2.7+
    from logging import NullHandler
except ImportError:
    class NullHandler(logging.Handler):
        def emit(self, record):
            pass
logging.basicConfig()
logging.getLogger(__name__).addHandler(NullHandler())
