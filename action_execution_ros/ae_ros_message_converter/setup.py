#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

dist_setup = generate_distutils_setup(
    packages=['ae_ros_message_converter'],
    package_dir={'ae_ros_message_converter': 'ros/src/ae_ros_message_converter'}
)

setup(**dist_setup)
