#!/bin/bash
# mkdir -p docker/third_party/assimp
# mkdir -p docker/third_party/glm
mkdir -p docker/third_party/perception_dep
# mkdir -p docker/third_party/sapien_dep
sudo apt-get install -y python3-pip curl
pip install gdown
# sapien prebuild binary
PROJECT_PATH=$(pwd)
# SAPIEN_BINARY=$(pwd)/docker/third_party/sapien_pack.tar.gz
# ASSIMP_BINARY=$(pwd)/docker/third_party/assimp/assimp.zip
# GLM_BINARY=$(pwd)/docker/third_party/glm/glm.zip
CERES_SOLVER_BINARY=$(pwd)/docker/third_party/perception_dep/ceres-solver.tar.gz
COLMAP_BINARY=$(pwd)/docker/third_party/perception_dep/colmap.tar.gz
GEOMETRY2_BINARY=$(pwd)/docker/third_party/perception_dep/geometry2.zip
# SPDLOG_BINARY=$(pwd)/docker/third_party/sapien_dep/spdlog.zip
# GLFW_BINARY=$(pwd)/docker/third_party/sapien_dep/glfw.zip

echo $
# if test -f "$SAPIEN_BINARY"; then
#     echo "$SAPIEN_BINARY exists. skip downloading"
# else 
#     echo "Download prebuilt sapien to $SAPIEN_BINARY"
#     gdown https://drive.google.com/uc?id=123iBL1yBaBRqmRHX7rvsgavIj8DkCuew -O docker/third_party/sapien_pack.tar.gz
#     # Extract the package to 
#     echo "Extract sapien package to $PROJECT_PATH/docker/third_party"
#     tar xvf $SAPIEN_BINARY --strip-components=1 -C $PROJECT_PATH/docker/third_party/
#     echo "Finish extract sapien package"
# fi

# download assimp
# if test -f "$ASSIMP_BINARY"; then
#     echo "$ASSIMP_BINARY exists. skip downloading"
# else
#     echo "Download assimp 5.0.1 to $ASSIMP_BINARY"
#     curl -L https://github.com/assimp/assimp/archive/refs/tags/v5.0.1.zip -o $ASSIMP_BINARY
# fi
# # download glm
# if test -f "$GLM_BINARY"; then
#     echo "$GLM_BINARY exists. skip downloading"
# else
#     echo "Download glm 0.9.9.8 to $GLM_BINARY"
#     curl -L https://github.com/g-truc/glm/archive/refs/tags/0.9.9.8.zip -o $GLM_BINARY
# fi
# download cere solver
if test -f "$CERES_SOLVER_BINARY"; then
    echo "$CERES_SOLVER_BINARY exists. skip downloading"
else
    echo "Download ceres 2.0.0 to $CERES_SOLVER_BINARY"
    curl -L http://ceres-solver.org/ceres-solver-2.0.0.tar.gz -o $CERES_SOLVER_BINARY
fi


# download colmap
if test -f "$COLMAP_BINARY"; then
    echo "$COLMAP_BINARY exists. skip downloading"
else
    echo "Download colmap 3.7 to $COLMAP_BINARY"
    curl -L https://github.com/colmap/colmap/archive/refs/tags/3.7.tar.gz -o $COLMAP_BINARY
fi

# if test -f "$SPDLOG_BINARY"; then
#     echo "$SPDLOG_BINARY exists. skip downloading"
# else
#     echo "Download spdlog v1.8.5 to $SPDLOG_BINARY"
#     curl -L https://github.com/gabime/spdlog/archive/refs/tags/v1.8.5.zip -o $SPDLOG_BINARY
# fi

# if test -f "$GLFW_BINARY"; then
#     echo "$GLFW_BINARY exists. skip downloading"
# else
#     echo "Download glfw 3.3.3 to $GLFW_BINARY"
#     curl -L https://github.com/glfw/glfw/archive/refs/tags/3.3.3.zip -o $GLFW_BINARY
# fi


# download geometry2    
if test -f "$GEOMETRY2_BINARY"; then
    echo "$GEOMETRY2_BINARY exists. skip downloading"
else
    echo "Download geometry2 noetic devel to $GEOMETRY2_BINARY"
    curl -L https://github.com/ros/geometry2/archive/refs/heads/noetic-devel.zip -o $GEOMETRY2_BINARY
fi