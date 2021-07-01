cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_msg)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

add_action_files(
  FILES
  Organization.action
)

add_message_files(
  FILES
  PerceptionResult.msg
)

add_service_files(
  FILES
  PerceptionTarget.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_perception
  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)
