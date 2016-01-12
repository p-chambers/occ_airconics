#!/bin/bash
# @Author: pchambers
# @Date:   2016-01-08 17:35:49
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-01-12 13:30:35

# Create the Conda test environment (note that pytest is required, otherwise wrong
# python is used and the module is not found!)
conda create --name occ_airconics_test python=2 pytest
source activate occ_airconics_test

# Install and Test the module, saving report as junitxml for jenkins
python setup.py install
py.test -v --junitxml=pytest-report.xml

# Clean the Conda test environment
source deactivate
conda env remove -n occ_airconics_test
