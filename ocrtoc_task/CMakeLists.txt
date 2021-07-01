cmake_minimum_required(VERSION 2.8.3)
project(ocrtoc_task)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_e_description
#  CATKIN_DEPENDS urdf
#  DEPENDS system_lib
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/get_6dpose_dump_simulator.py
  scripts/task_evaluation.py
  scripts/trigger.py
  scripts/grasp_detection.py
  launch/timed_roslaunch.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
