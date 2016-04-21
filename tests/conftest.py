# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 16:11:18 2016

@author: pchambers
"""
import pytest


def pytest_addoption(parser):
    parser.addoption("--examples", action="store_true",
                     help="run user example scripts")
