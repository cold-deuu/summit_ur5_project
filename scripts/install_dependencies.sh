#!/usr/bin/env bash
# Installs development/runtime dependencies only. Starts no ROS nodes or goals.
set -euo pipefail
if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  echo 'Install ROS 1 Noetic on Ubuntu 20.04 and configure its apt repository first.' >&2
  exit 1
fi
with_sim=false
if [[ ${1:-} == --with-sim ]]; then with_sim=true; shift; fi
if [[ $# != 0 ]]; then echo 'Usage: install_dependencies.sh [--with-sim]' >&2; exit 2; fi
sudo apt-get update
sudo apt-get install -y build-essential cmake git pkg-config python3-pip python3-rosdep \
  python3-numpy python3-scipy python3-matplotlib python3-yaml python3-rospkg python3-opencv \
  libeigen3-dev libboost-system-dev libccd-dev libgl1-mesa-dev libglu1-mesa-dev \
  libglew-dev libglfw3-dev libosmesa6-dev \
  ros-noetic-pinocchio ros-noetic-eiquadprog ros-noetic-fcl \
  ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-ur-robot-driver \
  ros-noetic-ur-calibration ros-noetic-moveit ros-noetic-joint-state-publisher-gui \
  ros-noetic-robot-state-publisher ros-noetic-tf-conversions ros-noetic-eigen-conversions \
  ros-noetic-xacro ros-noetic-gazebo-msgs ros-noetic-gazebo-plugins ros-noetic-hector-gazebo-plugins
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update --include-eol-distros
repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
# A clean shell is recommended; rosdep --ignore-src sees all bundled packages.
set +u
source /opt/ros/noetic/setup.bash
set -u
rosdep install --from-paths "$repo_dir" --ignore-src --rosdistro noetic -y
if $with_sim; then
  python3 -m pip install --user -r "$repo_dir/scripts/requirements-simulation.txt"
fi
echo 'Dependencies ready. Run catkin_make from the parent catkin workspace.'
