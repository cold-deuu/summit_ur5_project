cmake_minimum_required(VERSION 3.0.2)
project(summit_ur5e_moveit_config)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY config launch mobile_assets mobile_sim_assets DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/import_mobile_model.py scripts/check_mobile_mapping.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
