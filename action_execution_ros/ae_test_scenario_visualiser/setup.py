#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dist_setup = generate_distutils_setup(
    packages=['ae_test_scenario_visualiser'],
    package_dir={'ae_test_scenario_visualiser': 'ros/src/ae_test_scenario_visualiser'}
)

setup(**dist_setup)
