cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_common)

find_package(catkin REQUIRED COMPONENTS
   geometry_msgs
   message_generation
   rospy
   std_msgs
)

catkin_python_setup()

add_service_files(
  FILES
  JointSpaceGoal.srv
  ManipulatorInfo.srv
  ManipulatorState.srv
  PoseGoal.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_common
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/manipulator_interface_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
