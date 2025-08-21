cmake_minimum_required(VERSION 2.8.3)
project(rci_action_manager)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rci_holistic_controller REQUIRED)
find_package(sca_pkg REQUIRED)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  actionlib_msgs
  actionlib
  rci_holistic_controller
  sca_pkg
  message_generation
  std_msgs
  tf
)



ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

set(CMAKE_CXX_STANDARD 14)
###################################
## catkin specific configuration ##
###################################


add_action_files(DIRECTORY action FILES
  SE3.action PlatformSE3.action JointPosture.action FrankaSE3.action
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
 controller_interface
 pluginlib
 roscpp
 sensor_msgs
 std_msgs
 tf
 
)


###########
## Build ##
###########

include_directories(
    include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rci_holistic_controller_INCLUDE_DIRS}
  ${sca_pkg_INCLUDE_DIRS}
# ${husky_description_INCLUDE_DIRS}
  DEPENDS system_lib

)

install(PROGRAMS
	
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

# Library
SET(${PROJECT_NAME}_HEADERS
  #Simulator
  include/rci_action_manager/controller/controller.hpp

  #Server
  include/rci_action_manager/server/joint_posture_server.hpp
  include/rci_action_manager/server/wholebody_server.hpp
#  include/rci_action_manager/server/platform_se3_server.hpp
#  include/rci_action_manager/server/franka_se3_server.hpp
  include/rci_action_manager/server/action_server_base.hpp


)

SET(${PROJECT_NAME}_SOURCES
  #Controller
  src/controller/controller.cpp
  
  #Server
  src/server/wholebody_server.cpp
  #src/server/platform_se3_server.cpp
  #src/server/franka_se3_server.cpp
  src/server/joint_posture_server.cpp

)


SET(CMAKE_BUILD_TYPE "Release")


ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})

TARGET_INCLUDE_DIRECTORIES(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
  ${CMAKE_SOURCE_DIR}/include
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${rci_holistic_controller_LIBRARIES} ${sca_pkg_INCLUDE_DIRS} pinocchio::pinocchio eiquadprog::eiquadprog -lpthread)


add_executable(${PROJECT_NAME}_simulation src/simulation/simulation.cpp)


target_link_libraries(${PROJECT_NAME}_simulation ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

