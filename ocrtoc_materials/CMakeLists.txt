cmake_minimum_required(VERSION 3.0.2)
project(ocrtoc_materials)

find_package(catkin REQUIRED)

catkin_package()

include_directories()

install(DIRECTORY
  models
  scenes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
