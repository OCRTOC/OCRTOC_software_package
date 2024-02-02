#!/bin/bash
mkdir -p docker/third_party/assimp
mkdir -p docker/third_party/glm
mkdir -p docker/third_party/perception_dep
mkdir -p docker/third_party/sapien_dep
sudo apt-get install -y python3-pip 
pip install gdown
# sapien prebuild binary
PROJECT_PATH=$(pwd)
SAPIEN_BINARY=$(pwd)/docker/third_party/sapien_pack.tar.gz
ASSIMP_BINARY=$(pwd)/docker/third_party/assimp/assimp.zip
GLM_BINARY=$(pwd)/docker/third_party/glm/glm.zip
CERES_SOLVER_BINARY=$(pwd)/docker/third_party/perception_dep/ceres-solver.zip
COLMAP_BINARY=$(pwd)/docker/third_party/perception_dep/colmap.zip
GEOMETRY2_BINARY=$(pwd)/docker/third_party/perception_dep/geometry2.zip

echo $
if test -f "$SAPIEN_BINARY"; then
    echo "$SAPIEN_BINARY exists. skip downloading"
else 
    echo "Download prebuilt sapien to $SAPIEN_BINARY"
    gdown https://drive.google.com/uc?id=123iBL1yBaBRqmRHX7rvsgavIj8DkCuew -O docker/third_party/sapien_pack.tar.gz
fi
# Extract the package to 
echo "Extract sapien package to $PROJECT_PATH/docker/third_party"
tar xvf $SAPIEN_BINARY --strip-components=1 -C $PROJECT_PATH/docker/third_party/

# download assimp
if test -f "$ASSIMP_BINARY"; then
    echo "$ASSIMP_BINARY exists. skip downloading"
else
    echo "Download assimp 5.0.1 to $ASSIMP_BINARY"
    curl -L https://github.com/assimp/assimp/archive/refs/tags/v5.0.1.zip -O $ASSIMP_BINARY

# download glm
if test -f "$GLM_BINARY"; then
    echo "$GLM_BINARY exists. skip downloading"
else
    echo "Download glm 0.9.9.8 to $GLM_BINARY"
    curl -L https://github.com/g-truc/glm/archive/refs/tags/0.9.9.8.zip -O $GLM_BINARY

# download cere solver
if test -f "$CERES_SOLVER_BINARY"; then
    echo "$CERES_SOLVER_BINARY exists. skip downloading"
else
    echo "Download ceres 2.0.0 to $CERES_SOLVER_BINARY"
    curl -L http://ceres-solver.org/ceres-solver-2.0.0.tar.gz -O $CERES_SOLVER_BINARY

# download geometry2
if test -f "$GEOMETRY2_BINARY"; then
    echo "$GEOMETRY2_BINARY exists. skip downloading"
else
    echo "Download geometry2 noetic devel to $GEOMETRY2_BINARY"
    curl -L https://github.com/ros/geometry2/archive/refs/heads/noetic-devel.zip -O $GEOMETRY2_BINARY
