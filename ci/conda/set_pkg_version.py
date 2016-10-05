# -*- coding: utf-8 -*-
# @Author: p-chambers
# @Date:   2016-10-05 14:35:02
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-10-05 14:38:46

import os
from setuptools_scm import get_version


version = get_version(root='..', relative_to=__file__)

os.environ['OCC_AIRCONICS_VERSION'] = version
