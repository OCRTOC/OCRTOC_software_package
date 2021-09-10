## update: 2021-07-09
- Removed the duplicate files in pybullet_simulator.
- Updated the table mesh in pybullet_simulator.
- Updated the robot_description in pybullet_simulator.
- Added a readme about sapien.

## update: 2021-09-10
- Modify docker image from version 2.0 to 2.1
    - Change base image from `nvidia/cudagl:10.2-devel-ubuntu18.04` to `nvidia/cudagl:11.2.1-devel-ubuntu18.04`
    - NVIDIA 30xx GPU is supported now.
- Fix bug on pybullet simulator.
    - ROS interface will wait until the pybullet backend is ready. 