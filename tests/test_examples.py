# -*- coding: utf-8 -*-
"""
Created on Tue Apr 19 14:48:21 2016

@author: pchambers
"""
import os
import subprocess
import inspect
import pytest


@pytest.mark.skipif(not pytest.config.getvalue('--examples'),
                    reason="--examples was not specified")
def test_examples():
    """Runs the example scripts based on core QT viewer in
    OCC_Airconics/examples/core. examples directory is expected to be at the
    same level as this test directory. All .py files will be run

    Notes
    -----
    This function still currently requires visual inspection that the shapes
    are produced as expected. This may be changed to automated later.
    * This test can be skipped by py.test if necessary
    """
    import airconics
    # pytest runs test files in ./__pycache__: need to go up two levels
    example_dir = os.path.abspath(
        os.path.join(__file__, '..', '..', 'examples', 'core'))
    example_scripts = os.listdir(example_dir)
    for script in example_scripts:
        if script.endswith('.py'):
            fname = os.path.join(example_dir, script)
            try:
                subprocess.check_call(['python', fname])
            except subprocess.CalledProcessError:
                raise AssertionError('Example {} failed'.format(fname))
