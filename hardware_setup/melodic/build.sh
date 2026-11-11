#!/bin/bash
# Build only the HQP controller packages (no MoveIt / MuJoCo / BRL planner needed).
# Usage: build.sh [workspace]   default: the catkin workspace this repo is in
set -e
D=$(cd "$(dirname "$0")" && pwd)
WS=${1:-$(cd "$D/../../../.." && pwd)}
[ -d "$WS/src" ] || { echo "ERROR: $WS is not a catkin workspace (no src/). Usage: build.sh <workspace>"; exit 1; }
source /opt/ros/melodic/setup.bash
cd "$WS"
# Python 2.7 headers: pinocchio 2.6's CMake config looks for them.
catkin_make -DCATKIN_WHITELIST_PACKAGES="rci_holistic_controller;sca_pkg;rci_action_manager;brl_interfaces;summit_ur5_hardware_setup" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -j"$(nproc)"
echo; echo "DONE. In every new terminal: source $WS/devel/setup.bash   (then: check.sh, selftest.sh)"
