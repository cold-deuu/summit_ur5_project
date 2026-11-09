cmake_minimum_required(VERSION 2.8.3)
project(rci_action_manager)

set(CMAKE_CXX_FLAGS "-std=c++17")

find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(toppra REQUIRED)
find_package(rci_holistic_controller REQUIRED)
find_package(sca_pkg REQUIRED)

find_package(Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  controller_manager_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  actionlib_msgs
  actionlib
  rci_holistic_controller
  sca_pkg
  message_generation
  std_srvs
  std_msgs
  tf
)



ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)

set(CMAKE_CXX_STANDARD 17)
###################################
## catkin specific configuration ##
###################################


add_action_files(DIRECTORY action FILES
  SE3.action PlatformSE3.action JointPosture.action FrankaSE3.action Moveit.action
)
add_message_files(FILES MoveitOptimalStatus.msg ControlTelemetry.msg)
add_service_files(FILES SetAdmittanceMode.srv)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    trajectory_msgs
    actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
 controller_interface
 pluginlib
 roscpp
 nav_msgs
 controller_manager_msgs
 sensor_msgs
 trajectory_msgs
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
  include/rci_action_manager/server/moveit_server.hpp
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
  src/server/moveit_server.cpp

)


SET(CMAKE_BUILD_TYPE "Release")


ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

TARGET_INCLUDE_DIRECTORIES(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>
  ${CMAKE_SOURCE_DIR}/include
)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC ${rci_holistic_controller_LIBRARIES} ${sca_pkg_LIBRARIES} ${catkin_LIBRARIES} pinocchio::pinocchio eiquadprog::eiquadprog -lpthread)


add_executable(${PROJECT_NAME}_simulation src/simulation/simulation.cpp)
target_link_libraries(${PROJECT_NAME}_simulation ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_real src/real/real.cpp)
target_link_libraries(${PROJECT_NAME}_real ${PROJECT_NAME} eiquadprog::eiquadprog pinocchio::pinocchio ${catkin_LIBRARIES})
foreach(target ${PROJECT_NAME}_simulation ${PROJECT_NAME}_real)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()


install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_simulation ${PROJECT_NAME}_real
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


if(CATKIN_ENABLE_TESTING)
  add_executable(admittance_controller_test test/admittance_controller_test.cpp)
  add_dependencies(admittance_controller_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries(admittance_controller_test ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

add_library(moveit_optimal_common
  src/controller/moveit_optimal_controller.cpp
  src/controller/moveit_topic_node.cpp
  src/controller/path_retiming.cpp)
set_target_properties(moveit_optimal_common PROPERTIES CXX_STANDARD 17)
target_link_libraries(moveit_optimal_common ${PROJECT_NAME} toppra::toppra ${catkin_LIBRARIES})
add_dependencies(moveit_optimal_common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_executable(moveit_optimal_sim src/simulation/sim_moveit.cpp)
add_executable(moveit_optimal_real src/real/real_moveit.cpp)
foreach(target moveit_optimal_sim moveit_optimal_real)
  target_link_libraries(${target} moveit_optimal_common)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS})
endforeach()
install(TARGETS moveit_optimal_common moveit_optimal_sim moveit_optimal_real
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/simulation/moveit_mujoco_backend.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  add_library(moveit_allocation_guard SHARED test/allocation_guard.c)
  add_executable(moveit_core_test test/moveit_core_test.cpp)
  set_target_properties(moveit_core_test PROPERTIES CXX_STANDARD 17)
  target_link_libraries(moveit_core_test moveit_optimal_common moveit_allocation_guard)
  add_test(NAME moveit_core_test COMMAND moveit_core_test
    ${CMAKE_CURRENT_SOURCE_DIR}/../summit_ur5e_moveit_config/config/ur5e.urdf)
endif()
catkin_install_python(PROGRAMS scripts/test_moveit_optimal.py
  scripts/mm_client.py scripts/simulation/mujoco_launch.py
  scripts/test_moveit_long_tracking.py scripts/test_moveit_hold_push.py scripts/render_moveit_validation.py
  scripts/test_mobile_actions.py scripts/render_mobile_actions.py scripts/ur_wrench_to_wrist.py scripts/log_action_telemetry.py scripts/plot_action_telemetry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
