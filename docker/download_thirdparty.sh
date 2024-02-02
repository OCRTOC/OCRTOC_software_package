#!/bin/bash
mkdir -p docker/third_party/assimp
mkdir -p docker/third_party/glm
mkdir -p docker/third_party/perception_dep
mkdir -p docker/third_party/sapien_dep
sudo apt-get install -y python3-pip 
pip install gdown
# sapien prebuild binary
gdown https://drive.google.com/uc?id=123iBL1yBaBRqmRHX7rvsgavIj8DkCuew -O docker/third_party/sapien_build.zip
# 
