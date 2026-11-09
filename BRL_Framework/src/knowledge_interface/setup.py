#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Setup script for knowledge_interface ROS package.

Standard ROS Python package structure:
    src/knowledge_interface/
        ├── __init__.py
        └── scene_graph/
            └── ...
"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# ROS catkin setup
# Note: catkin doesn't support 'entry_points' or 'zip_safe'
d = generate_distutils_setup(
    packages=['knowledge_interface', 'knowledge_interface.scene_graph'],
    package_dir={'': 'src'},
)

setup(**d)
