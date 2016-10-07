#!/bin/bash
# @Author: pchambers
# @Date:   2016-01-08 17:35:49
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-10-07 16:12:44

# Create the Conda test environment (note that pytest is required, otherwise wrong
# python is used and the module is not found!)
conda create --name occ_airconics_test python=2 pytest
source activate occ_airconics_test

# Install the python-occ precompiled binary from DLR-SC with Conda
conda install --name occ_airconics_test -c https://conda.anaconda.org/dlr-sc pythonocc-core

# Install and Test the module, saving report as junitxml for jenkins
pip install ./
py.test -v --junitxml=pytest-report.xml


# Clean the Conda test environment
source deactivate
conda env remove -n occ_airconics_test
