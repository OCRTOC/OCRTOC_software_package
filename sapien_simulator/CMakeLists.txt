cmake_minimum_required(VERSION 3.0.2)
project(sapien_simulator)

find_package(catkin REQUIRED)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES ur_e_description
        #  CATKIN_DEPENDS urdf
        #  DEPENDS system_lib
)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/sapien_env.py
  scripts/asset_utils.py
  scripts/camera_info_pub.py
  scripts/robot_const.py
  scripts/viz_urdf.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
