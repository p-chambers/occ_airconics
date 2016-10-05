#!/bin/bash

# Add DLR-SC channel if not present ?
# conda config --add channels https://conda.anaconda.org/dlr-sc

# Set environment variable for pkg version using setuptools_scm functions,
# incrementing latest tag by 0.1
pwd
python ./ci/conda/set_pkg_version.py

echo 'Building occ_airconics version {$OCC_AIRCONICS_VERSION}'

$PYTHON setup.py install

# Add more build steps here, if they are necessary.

# See
# http://docs.continuum.io/conda/build.html
# for a list of environment variables that are set during the build process.
