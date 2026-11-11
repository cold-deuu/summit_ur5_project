#!/bin/bash
# Ubuntu 18.04 + ROS Melodic: system libraries for the HQP controller. Safe to re-run
# (anything already at the right version is skipped).
#   install_deps.sh              apt packages + fcl/toppra/eiquadprog from source (needs internet)
#   install_deps.sh --fetch DIR  only download the three sources into DIR (any PC with internet;
#                                copy DIR by USB to the field PC)
#   install_deps.sh --src DIR    build from the sources in DIR (no download)
# Versions = the Noetic build: fcl 0.6.1, toppra 0.6.4, eiquadprog 1.2.9 (pinocchio: apt 2.6).
set -e
SRC=$HOME/hqp_deps_src; FETCH_ONLY=0; NO_DOWNLOAD=0
case "$1" in
  --fetch) SRC=${2:?--fetch DIR}; FETCH_ONLY=1 ;;
  --src)   SRC=${2:?--src DIR}; NO_DOWNLOAD=1 ;;
  "") ;;
  *) sed -n '2,9p' "$0"; exit 1 ;;
esac
SRC=$(mkdir -p "$SRC" && cd "$SRC" && pwd)
say() { echo; echo "=== $*"; }

fetch() {  # name tag url
  if [ -d "$SRC/$1/.git" ] || [ -f "$SRC/$1/CMakeLists.txt" ]; then echo "$1: source present in $SRC/$1"; return; fi
  [ $NO_DOWNLOAD = 1 ] && { echo "ERROR: $SRC/$1 missing (copy it from a --fetch DIR)"; exit 1; }
  git clone --depth 1 --recursive --branch "$2" "$3" "$SRC/$1"
}
fetch_all() {
  fetch fcl 0.6.1 https://github.com/flexible-collision-library/fcl.git
  fetch toppra v0.6.4 https://github.com/hungpham2511/toppra.git
  fetch eiquadprog v1.2.9 https://github.com/stack-of-tasks/eiquadprog.git
}
if [ $FETCH_ONLY = 1 ]; then say "download sources -> $SRC"; fetch_all; echo; echo "Copy $SRC to the field PC, then: install_deps.sh --src <copied dir>"; exit 0; fi

. /etc/os-release
[ "$VERSION_ID" = 18.04 ] || echo "WARNING: this script is for Ubuntu 18.04 (found $VERSION_ID)"
[ -d /opt/ros/melodic ] || { echo "ERROR: /opt/ros/melodic not found; install ROS Melodic first"; exit 1; }

say "apt packages"
sudo apt-get update || echo "WARNING: apt-get update failed (offline?); trying with the current package lists"
sudo apt-get install -y --no-install-recommends \
  build-essential cmake git pkg-config libeigen3-dev libboost-system-dev libccd-dev liboctomap-dev \
  python-dev python-numpy python-yaml \
  ros-melodic-pinocchio ros-melodic-tf ros-melodic-tf-conversions ros-melodic-actionlib ros-melodic-actionlib-msgs \
  ros-melodic-controller-manager ros-melodic-controller-manager-msgs ros-melodic-nav-msgs ros-melodic-trajectory-msgs \
  ros-melodic-std-srvs ros-melodic-gazebo-msgs ros-melodic-sensor-msgs ros-melodic-geometry-msgs ros-melodic-roslib \
  ros-melodic-image-transport ros-melodic-compressed-depth-image-transport

say "ros-melodic-eiquadprog (1.2.5) must not shadow 1.2.9"
if dpkg -s ros-melodic-eiquadprog >/dev/null 2>&1; then
  others=$(apt-get -s remove ros-melodic-eiquadprog | awk '/^Remv/{print $2}' | grep -vx ros-melodic-eiquadprog || true)
  if [ -n "$others" ]; then
    echo "ERROR: removing ros-melodic-eiquadprog would also remove: $others"
    echo "       Ask before going on; the controller build needs eiquadprog 1.2.9 from /usr/local."; exit 1
  fi
  sudo apt-get remove -y ros-melodic-eiquadprog
else echo "not installed: OK"; fi

version() {  # installed /usr/local version of a CMake package
  grep -ho 'PACKAGE_VERSION "[0-9.]*"' /usr/local/lib/cmake/$1/*ersion.cmake 2>/dev/null | head -1 | grep -o '[0-9.]*' || true
}
build() {  # name version source-subdir cmake-args...
  local name=$1 want=$2 sub=$3; shift 3
  if [ "$(version $name)" = "$want" ]; then echo "$name $want: already in /usr/local, skipped"; return; fi
  rm -rf "$SRC/$name/build"; mkdir -p "$SRC/$name/build"; cd "$SRC/$name/build"
  cmake "../$sub" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local "$@"
  make -j"$(nproc)"; sudo make install; sudo ldconfig
  [ "$(version $name)" = "$want" ] || { echo "ERROR: $name installed but version is '$(version $name)', expected $want"; exit 1; }
  echo "$name $want: installed"
}
say "sources ($SRC)"; fetch_all
# toppra 0.6.4 asks for CMake 3.16 but uses nothing newer than 18.04's 3.10: lower only that line.
sed -i 's/cmake_minimum_required(VERSION 3.16)/cmake_minimum_required(VERSION 3.10)/' "$SRC/toppra/cpp/CMakeLists.txt"
say "fcl 0.6.1";       build fcl 0.6.1 . -DBUILD_TESTING=OFF -DFCL_WITH_OCTOMAP=OFF
say "toppra 0.6.4";    build toppra 0.6.4 cpp -DBUILD_TESTS=OFF -DPYTHON_BINDINGS=OFF -DTOPPRA_WARN_ON=OFF
say "eiquadprog 1.2.9"; build eiquadprog 1.2.9 . -DBUILD_TESTING=OFF
echo; echo "DONE. Next: build.sh, then check.sh"
