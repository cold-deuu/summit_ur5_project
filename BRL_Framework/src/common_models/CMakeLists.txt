cmake_minimum_required(VERSION 3.0.2)
project(common_models)

find_package(catkin REQUIRED COMPONENTS
  rospy
  brl_interfaces
  geometry_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy brl_interfaces geometry_msgs
)

# Install Python modules
catkin_install_python(PROGRAMS
  # No scripts currently
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)