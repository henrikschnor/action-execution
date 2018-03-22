#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dist_setup = generate_distutils_setup(
    packages=['ae_data_visualiser'],
    package_dir={'ae_data_visualiser': 'ros/src/ae_data_visualiser'}
)

setup(**dist_setup)
