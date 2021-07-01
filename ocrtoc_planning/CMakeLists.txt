cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_planning)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  ocrtoc_msg
  ocrtoc_common
  gazebo_msgs
  control_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_planning
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs message_generation roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/motion_planning.py
  scripts/primitives.py
  scripts/solution_entrance.py
  scripts/task_planning.py
  scripts/viewer.py
  scripts/domain.pddl
  scripts/stream.pddl
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY
  scripts/pddlstream
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)