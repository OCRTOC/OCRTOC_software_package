# Install Pytorch 
python3 -m pip --default-timeout=1000000000 install torch==1.8.2+cu111 torchvision==0.9.2+cu111 -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html

# Rebuild ocrtoc ros
cd /root/ocrtoc_ws/
rm -rf build devel
catkin_make -j 8
source /root/ocrtoc_ws/devel/setup.bash

# install package for graspnet baseline
#update-alternatives --set gcc /usr/bin/gcc-9
roscd ocrtoc_perception/src/ocrtoc_perception/graspnet
cd pointnet2
#python3 setup.py install
python3 setup.py install --install-lib /usr/lib/python3/dist-packages

cd ../knn
#python3 setup.py install
python3 setup.py install --install-lib /usr/lib/python3/dist-packages

# # install package for pddlstream
roscd ocrtoc_planning/scripts/pddlstream
# ./FastDownward/build.py release64
./downward/build.py
# update-alternatives --set gcc /usr/bin/gcc-9

source /root/ocrtoc_ws/devel/setup.bash

# generate bounding boxes for evaluation
roscd ocrtoc_perception/src/ocrtoc_perception/pose
python3 bbox_generation.py