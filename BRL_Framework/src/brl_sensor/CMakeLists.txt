cmake_minimum_required(VERSION 3.0.2)
project(brl_sensor)

# =============================================================================
# Dependencies
# =============================================================================

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf
  brl_interfaces
  common_models
)

# =============================================================================
# Catkin Setup
# =============================================================================

# Python 패키지 설정 (setup.py 사용)
catkin_python_setup()

# 패키지 선언
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    tf
    brl_interfaces
    common_models
)

# =============================================================================
# Include Directories
# =============================================================================

include_directories(${catkin_INCLUDE_DIRS})

# =============================================================================
# Install
# =============================================================================

# Python 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/sensor_node.py
  scripts/dummy_sensor_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
