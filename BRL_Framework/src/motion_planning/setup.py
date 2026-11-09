#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup script for motion_planning ROS package.

Standard ROS Python package structure:
    src/motion_planning/
        ├── __init__.py
        ├── motion_planner.py
        └── visual_servoing.py
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['motion_planning'],
    package_dir={'': 'src'},
)

setup(**d)
