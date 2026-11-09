#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup script for brl_sensor ROS package.

Standard ROS Python package structure:
    src/brl_sensor/
        ├── __init__.py
        ├── robot_tracker.py
        └── tomato_detector.py
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ROS catkin setup
d = generate_distutils_setup(
    packages=['brl_sensor'],
    package_dir={'': 'src'},
)

setup(**d)
