# checkout realsense driver
cd /root/ocrtoc_ws/src/drivers/realsense-ros
git checkout 2.3.0

# rebuild ocrtoc ros
cd /root/ocrtoc_ws
rm -rf build devel
catkin_make -j 8
source /root/ocrtoc_ws/devel/setup.bash

# install package for graspnet baseline
update-alternatives --set gcc /usr/bin/gcc-7
roscd ocrtoc_perception/src/ocrtoc_perception/graspnet
cd pointnet2
python3 setup.py install
cd ../knn
python3 setup.py install

# install package for pddlstream
roscd ocrtoc_planning/scripts/pddlstream
./FastDownward/build.py release64
update-alternatives --set gcc /usr/bin/gcc-9

source /root/ocrtoc_ws/devel/setup.bash
