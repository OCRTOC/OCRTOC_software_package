cmake_minimum_required(VERSION 3.0.2)
project(pybullet_simulator)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/load_env_ros.py
  scripts/pybullet_env.py
  scripts/ros_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
