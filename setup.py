# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 16:13:40 2015

@author: pchambers
"""
from setuptools import setup, find_packages
import os
import subprocess

# Versioning note: I didn't use scm_tools here as I wanted the release to be
# exactly equal to the latest git tag (releasing a new tag updates the version,
# which will in turn be uploaded to anaconda)
git_tag_cmd = "git describe --tags --abbrev=0 | tr -d 'v'"
comm = subprocess.Popen(git_tag_cmd, shell=True, stdout=subprocess.PIPE)
version = comm.communicate()[0].strip().decode("utf-8")


setup(
    name='airconics',
    version=version,
    description='Scripted Aircraft Geometry Module based on Python-OCC',
    packages=find_packages(),
    include_package_data=True,
    # package_data={'': ['*.dat']},
    install_requires=["numpy", "pydot"],
    author='Paul Chambers, Andras Sobester',
    author_email='P.R.Chambers@soton.ac.uk, A.Sobester@soton.ac.uk',
)
