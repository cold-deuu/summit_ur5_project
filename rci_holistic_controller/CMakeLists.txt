cmake_minimum_required(VERSION 3.0.2)
project(rci_holistic_controller)
add_compile_options(-std=c++17 -Ofast)

SET(CXX_DISABLE_WERROR TRUE)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)
SET(PROJECT_USE_CMAKE_EXPORT TRUE)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)
find_package(eiquadprog REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf 
 )


find_package(catkin REQUIRED)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME} 
 CATKIN_DEPENDS 
)

include_directories(include )
ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)


# Main Library
SET(${PROJECT_NAME}_MATH_HEADERS
  include/rci_holistic_controller/math/math.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  include/rci_holistic_controller/trajectory/SE3_cubic.hpp
  include/rci_holistic_controller/trajectory/joint_cubic.hpp
  include/rci_holistic_controller/trajectory/joint_cubic_array.hpp
  )


SET(${PROJECT_NAME}_ROBOTS_HEADERS
  include/rci_holistic_controller/robot/robot.hpp
  )

SET(${PROJECT_NAME}_CONSTRAINTS_HEADERS
  include/rci_holistic_controller/constraint/constraint_base.hpp
  include/rci_holistic_controller/constraint/constraint_equality.hpp
  include/rci_holistic_controller/constraint/constraint_inequality.hpp
  )

SET(${PROJECT_NAME}_TASK_HEADERS
  include/rci_holistic_controller/task/task_base.hpp
  include/rci_holistic_controller/task/task_se3_equality.hpp
  include/rci_holistic_controller/task/task_joint_posture_equality.hpp
  include/rci_holistic_controller/task/task_joint_velocity_limit_inequality.hpp
  include/rci_holistic_controller/task/task_sca_inequality.hpp
  include/rci_holistic_controller/task/task_base_fix_equality.hpp
  include/rci_holistic_controller/task/tasks.hpp

  
  )
SET(${PROJECT_NAME}_DATA_HEADERS
  include/rci_holistic_controller/data/hqp_data.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  ${${PROJECT_NAME}_MATH_HEADERS}
  ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
  ${${PROJECT_NAME}_ROBOTS_HEADERS}  
  ${${PROJECT_NAME}_CONSTRAINTS_HEADERS}  
  ${${PROJECT_NAME}_TASK_HEADERS}  
  ${${PROJECT_NAME}_DATA_HEADERS}  

  )

LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_MATH_SOURCES
  src/math/math.cpp
  )


SET(${PROJECT_NAME}_TRAJECTORIES_SOURCES
  src/trajectory/SE3_cubic.cpp
  src/trajectory/joint_cubic.cpp
  src/trajectory/joint_cubic_array.cpp
  )



SET(${PROJECT_NAME}_ROBOTS_SOURCES
  src/robot/robot.cpp
  )

SET(${PROJECT_NAME}_CONSTRAINTS_SOURCES
  src/constraint/constraint_equality.cpp
  src/constraint/constraint_inequality.cpp
  src/constraint/constraint_base.cpp
  )
  
SET(${PROJECT_NAME}_TASK_SOURCES
  src/task/task_base.cpp
  src/task/task_se3_equality.cpp
  src/task/task_joint_posture_equality.cpp
  src/task/task_joint_velocity_limit_inequality.cpp
  src/task/task_sca_inequality.cpp
  src/task/task_base_fix_equality.cpp
  )

SET(${PROJECT_NAME}_DATA_SOURCES
  src/data/hqp_data.cpp
  )

SET(${PROJECT_NAME}_SOURCES
  ${${PROJECT_NAME}_MATH_SOURCES}
  ${${PROJECT_NAME}_ROBOTS_SOURCES}
  ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
  ${${PROJECT_NAME}_CONSTRAINTS_SOURCES}
  ${${PROJECT_NAME}_TASK_SOURCES}
  ${${PROJECT_NAME}_DATA_SOURCES}
  )


SET(CMAKE_BUILD_TYPE "Release")

ADD_LIBRARY(${PROJECT_NAME} SHARED  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include> )
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC eiquadprog::eiquadprog pinocchio::pinocchio)


# Keep NaN/Inf failure checks effective despite the target's -Ofast.
set_source_files_properties(src/data/hqp_data.cpp PROPERTIES COMPILE_FLAGS "-fno-finite-math-only")

if(CATKIN_ENABLE_TESTING)
  add_executable(hqp_data_test test/hqp_data_test.cpp)
  target_include_directories(hqp_data_test PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(hqp_data_test ${PROJECT_NAME} ${catkin_LIBRARIES})
  add_test(NAME hqp_data_test COMMAND hqp_data_test)
endif()
