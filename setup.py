# -*- coding: utf-8 -*-
"""
Created on Fri Dec  4 16:13:40 2015

@author: pchambers
"""

#from distutils.core import setup
from setuptools import setup, find_packages
import os

setup(
    name='airconics',
    version="0.1",
    description='Scripted Aircraft Geometry Module based on Python-OCC',
    packages=find_packages(),
    include_package_data=True,
    # package_data={'': ['*.dat']},
    install_requires=["numpy"],
    author='Paul Chambers, Andras Sobester',
    author_email='P.R.Chambers@soton.ac.uk, A.Sobester@soton.ac.uk',
)
