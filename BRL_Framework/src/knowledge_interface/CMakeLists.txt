cmake_minimum_required(VERSION 3.0.2)
project(knowledge_interface)

# =============================================================================
# Dependencies
# =============================================================================

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  common_models
  brl_interfaces
)

# =============================================================================
# Catkin Setup
# =============================================================================

# Python 패키지 설정 (setup.py 사용)
catkin_python_setup()

# 메시지 생성 의존성
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  brl_interfaces
)

# 패키지 선언
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
    common_models
    brl_interfaces
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
  scripts/knowledge_graph_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
