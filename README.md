# SummitUR Whole-body Controller
This repository contains a whole-body controller for a Summit XLs mobile base with a UR5e manipulator, validated in the MuJoCo simulator.

## Overview
![Demo](doc/gif/se3_wbc.gif)
This project implements an optimal controller for a mobile manipulator using the Hierarchical Quadratic Programming (HQP) framework.

## Prerequisites
Before you begin, ensure you have the following dependencies installed:

1. **ROS 1 Noetic**: Please follow the official installation instructions.
2. **pinocchio** :
   For details, See the [official documents](https://github.com/stack-of-tasks/pinocchio). 
     ```bash
     sudo apt install ros-noetic-pinocchio
     ```
4. 
