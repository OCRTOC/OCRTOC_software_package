cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_perception)

find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   geometry_msgs
   std_msgs
)
catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ocrtoc_perception
#  CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/perception_entrance.py
  scripts/perception_server.py
  test/perception_target_client.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
