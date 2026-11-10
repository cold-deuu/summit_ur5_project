cmake_minimum_required(VERSION 3.0.2)
project(rci_action_manager)
# Message-only copy for the field PC. Keep msg/MoveitOptimalStatus.msg identical to
# ../../rci_action_manager/msg/MoveitOptimalStatus.msg (the md5 must match the HQP node).
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
add_message_files(FILES MoveitOptimalStatus.msg)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
