#!/bin/bash
# @Author: p-chambers
# @Date:   2016-10-05 18:11:19
# 
# I probably should have done this with Jenkins pipelines, but this script is
# used by a separate Jenkins job to build and upload the conda package when
# a new tag is released in master. 
#
# @Last Modified by:   p-chambers
# @Last Modified time: 2016-10-07 16:12:52

##############################################################################
## THIS PART IS THE SAME AS JENKINS_BUILD.SH, BUT IS INCLUDED HERE RATHER THAN
## IN JENKINS TO BE EXPLICIT. THE ENVIRONMENT NAME IS DIFFERENT.
##############################################################################
# Create the Conda test environment (note that pytest is required, otherwise
# wrong python is used and the module is not found!)
conda create --name occ_airconics_build python=2 pytest
source activate occ_airconics_build

# Install the python-occ precompiled binary from DLR-SC with Conda
conda install --name occ_airconics_build -c https://conda.anaconda.org/dlr-sc pythonocc-core

##############################################################################

conda list

# Build the conda module (note: uses ~/anacondaX/conda-bld/work)
conda build ./ci/conda

# UPDATE THIS TO THE PATH OF ANACONDA ON JENKINS SERVER: should be home
CONDA_PREFIX=~
echo "Anaconda path: ${CONDA_PREFIX}"

CONDA_BUILD_DIR=${CONDA_PREFIX}/conda-bld
echo "Conda build path: ${CONDA_BUILD_DIR}"

PKG_OUTPUT_DIR=./conda-output

mkdir $PKG_OUTPUT_DIR

# Obtain the tag name so we know which .tar.bz to look for
GIT_DESCRIBE_TAG="$(git describe --tags --abbrev=0 | tr -d 'v')"

# CHANGE THIS TO THE PLATFORM RUNNING ON JENKINS SERVER
JEKINS_PLATFORM=linux-64

# The conda build flag for specifying python versions to build for:
CONDA_PYVERSION_FLAGS="--py 2.7 --py 3.5"

# My jenkins is currently running on linux-64, so get the appropriate file:
CONDA_PKG_NAME=$CONDA_BUILD_DIR/${JEKINS_PLATFORM}/occ_airconics-${GIT_DESCRIBE_TAG}-py27*

echo "Making package ${CONDA_PKG_NAME_}"

# Install and test
conda install --use-local $CONDA_PKG_NAME

echo 'Preparing to run tests'
py.test -v --junitxml=pytest-report.xml

# Convert to all platforms and output in the current folder
conda convert --platform all $CONDA_PKG_NAME -o $PKG_OUTPUT_DIR

# Clean the Conda test/build environment
source deactivate
conda env remove -n occ_airconics_build

# clean conda build directories
conda build purge



# Also test the package on python 3:
conda create --name occ_airconics_py3build python=3 pytest
source activate occ_airconics_py3build

# Install the python-occ precompiled binary from DLR-SC with Conda
conda install --name occ_airconics_py3build -c https://conda.anaconda.org/dlr-sc pythonocc-core

CONDA_PKG_NAME=$CONDA_BUILD_DIR/${JEKINS_PLATFORM}/occ_airconics-${GIT_DESCRIBE_TAG}-py35*

# Install and test
conda install --use-local $CONDA_PKG_NAME

echo 'Preparing to run tests'
py.test -v --junitxml=pytest-report.xml

# Convert to all platforms and output in the current folder
conda convert --platform all $CONDA_PKG_NAME -o $PKG_OUTPUT_DIR

# Clean the Conda test/build python 3 environment
source deactivate
conda env remove -n occ_airconics_py3build
