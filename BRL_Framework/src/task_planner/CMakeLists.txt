cmake_minimum_required(VERSION 3.0.2)
project(task_planner)

# =============================================================================
# Dependencies
# =============================================================================

find_package(catkin REQUIRED COMPONENTS
  rospy
  actionlib
  std_srvs
  std_msgs
  common_models
  brl_interfaces
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
  actionlib
  std_srvs
    std_msgs
    common_models
    brl_interfaces
)

# =============================================================================
# Install
# =============================================================================

# Python 스크립트 설치
catkin_install_python(PROGRAMS
  scripts/task_planner_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch templates config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
