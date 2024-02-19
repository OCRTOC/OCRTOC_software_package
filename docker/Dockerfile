from nvidia/cudagl:11.4.2-devel-ubuntu20.04

ENV TZ=Asia/Shanghai
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


# Install Ubuntu tools.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && rm /etc/apt/sources.list.d/cuda.list \
    && apt-get update \
    && apt-get install -y build-essential git libssl-dev \
        lsb-release iputils-ping manpages-dev net-tools \
        python3-pip python3-tk \
        psmisc software-properties-common tree vim wget zip \
    # Install gl/x package for visualization inside docker
    && apt-get install -y libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
    # # Install gcc9
    # && add-apt-repository -y ppa:ubuntu-toolchain-r/test \
    # && apt-get update \
    && apt install -y gcc-9 g++-9 \
    # Install cmake 3.16.3
    && cd /root && wget --no-check-certificate https://cmake.org/files/v3.16/cmake-3.16.3.tar.gz \
    && tar -xf cmake-3.16.3.tar.gz && cd cmake-3.16.3 \
    && ./bootstrap && make -j4 && make install \
    && cd /root && rm -rf cmake-3.16.3 cmake-3.16.3.tar.gz

# Install ROS.
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt install -y curl \
    && curl -k -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt update \
    && apt-get update --fix-missing \
    && apt install -y ros-noetic-desktop-full \
    && echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc \
    && apt install -y python3-rosdep


# Install ROS dependencies.
RUN pip3 install catkin_pkg numpy transforms3d rospkg \
    && apt-get update && apt-get install -y alien \
    && apt-get install -y ros-noetic-ddynamic-reconfigure \
                          ros-noetic-effort-controllers \
                          ros-noetic-franka-ros \
                          ros-noetic-gripper-action-controller \
                          ros-noetic-joint-state-controller \
                          ros-noetic-joint-trajectory-controller \
                          ros-noetic-industrial-robot-status-interface \
                          ros-noetic-libfranka \
                          ros-noetic-moveit \
                          ros-noetic-position-controllers \
                          ros-noetic-realsense2-camera \
                          ros-noetic-ros-control \
                          ros-noetic-ros-controllers \
                          ros-noetic-ros-numpy \
                          ros-noetic-socketcan-interface \
                          ros-noetic-soem \
                          ros-noetic-ur-msgs \
                          ros-noetic-visp-hand2eye-calibration

# # Install drivers.
# RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - \
#     && apt-add-repository -y https://packages.microsoft.com/ubuntu/20.04/prod \
#     && apt-get update \
#     && echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections \
#     && echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections \
#     && apt install -y k4a-tools libk4a1.4-dev \
#     && apt-get install python-pymodbus socat \
#     && apt-key adv --keyserver keys.gnupg.net \
#         --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv \
#         --keyserver hkp://keyserver.ubuntu.com:80 \
#         --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# RUN git clone https://github.com/IntelRealSense/librealsense.git --recursive \
#     && cd librealsense \
#     && apt update \
#     && apt -y install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
#     libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at\
#     && mkdir build \
#     && cd build \
#     && cmake .. -DCMAKE_BUILD_TYPE=release \
#     && make -j8 \
#     && make install

# RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u \
#     && apt update \
#     && apt-get -y install librealsense2-dkms librealsense2-utils \
#         librealsense2-dev librealsense2-dbg

# Install Pybullet.
RUN python3 -m pip install pybullet==3.2.5 pyquaternion \
    && python3 -m pip install pip --upgrade  \
    && python3 -m pip install numpy --upgrade  \
    && python3 -m pip install scipy  \
    && apt update \
    && apt -y install htop imagemagick




# Install Sapien dependences.
COPY docker/third_party/assimp/assimp.zip /root
COPY docker/third_party/glm/glm.zip /root

RUN wget --no-check-certificate -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - \
    && wget --no-check-certificate -qO /etc/apt/sources.list.d/lunarg-vulkan-1.2.170-bionic.list https://packages.lunarg.com/vulkan/1.2.170/lunarg-vulkan-1.2.170-bionic.list \
    && apt-get -o "Acquire::https::Verify-Peer=false" update \
    && apt-get -o "Acquire::https::Verify-Peer=false" install -y vulkan-sdk libglew-dev \
    && cd /root  && unzip -d ./ assimp.zip && rm assimp.zip && cd assimp-5.0.1 &&  mkdir build && cd build \
    && cmake .. && make -j 8 && make install \
    && cd /root && rm -rf assimp-5.0.1 && unzip -d ./ glm.zip && rm glm.zip \
    && cd glm-0.9.9.8 &&  cp -r glm /usr/local/include \
    && cd /root && rm -rf glm-0.9.9.8

COPY docker/third_party/sapien_dep/spdlog.zip /root
COPY docker/third_party/sapien_dep/glfw.zip /root

RUN cd /root \
    && unzip -d ./  glfw.zip && rm glfw.zip && cd glfw-3.3.3 \
    && mkdir build && cd build && cmake .. && make install -j8 \
    && cd /root \    
    && unzip -d ./ spdlog.zip && rm spdlog.zip && cd spdlog-1.8.5 \
    && mkdir build && cd build && cmake .. && make install -j8 \
    && cd /root && rm -rf spdlog-1.8.5 && rm -rf glfw-3.3.3

# For ocrtoc_common.
RUN cd /root && mkdir -p workspace && cd workspace  && mkdir -p geometry2_ws
COPY docker/third_party/perception_dep/geometry2.zip /root/workspace/geometry2_ws
RUN cd /root/workspace/geometry2_ws  && unzip -d ./ geometry2.zip && rm geometry2.zip \
    && mv geometry2-noetic-devel src \
    && source /opt/ros/noetic/setup.bash \
    && pip install transforms3d \
    && pip3 install empy pyyaml virtualenv \
    && virtualenv -p /usr/bin/python3 venv \
    && source venv/bin/activate \
    && source /opt/ros/noetic/setup.bash \
    && catkin_make --cmake-args \
         -DCMAKE_BUILD_TYPE=Release \
         -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    && cd devel/lib/python3/dist-packages \
    && cp -r * /usr/local/lib/python3.8/dist-packages/

# For perception.
RUN python3 -m pip install pip --upgrade \
    && python3 -m pip install scikit-build Cython \
    && python3 -m pip install sklearn==0.0 \
    && python3 -m pip install --upgrade setuptools \
    && python3 -m pip install open3d==0.12.0 \
    && python3 -m pip install matplotlib graspnetAPI open3d-plus tensorboard==2.3 \
    && python3 -m pip install pyrealsense2 pyrender trimesh plyfile scikit-learn pytz \
    && python3 -m pip install imageio==2.5.0 pypng==0.0.20 \
         PyOpenGL==3.1.0 \
         triangle==20190115.2 glumpy==1.1.0 \
         Pillow==7.2.0 rtree 
    # && python3 -m pip --default-timeout=1000000000 install torch==1.8.2+cu111 torchvision==0.9.2+cu111 \
    #  -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html

    # For perception.
RUN apt -y install libboost-program-options-dev \
    libboost-filesystem-dev libboost-graph-dev libboost-system-dev \
    libboost-test-dev libeigen3-dev libsuitesparse-dev libfreeimage-dev \
    libgoogle-glog-dev libgflags-dev qtbase5-dev libqt5opengl5-dev \
    libcgal-dev libcgal-qt5-dev libatlas-base-dev libsuitesparse-dev

COPY docker/third_party/perception_dep/ceres-solver.tar.gz /root
COPY docker/third_party/perception_dep/colmap.tar.gz /root

RUN cd /root && tar -xf ceres-solver.tar.gz -C ./ && rm ceres-solver.tar.gz \
    && cd ceres-solver-2.0.0 \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make -j8 && make install \
    && cd ../.. && rm -rf ceres-solver-2.0.0 \
    && tar -xf colmap.tar.gz -C ./  && rm colmap.tar.gz \
    && cd colmap-3.7 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_C_COMPILER=/usr/bin/gcc-9 && make -j8 && make install \
    && cd ../.. && rm -rf colmap-3.7 \
    && python3 -m pip install pycolmap

# Build Sapien.
COPY docker/third_party/sapien_pack /root/workspace/sapien_pack
RUN cd /root/workspace/sapien_pack && cp nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json \
    && source /opt/ros/noetic/setup.bash \
    && cd sapien_ros && python3 setup.py bdist_wheel \
    && pip3 install dist/sapien_ros1-1.0.0rc2-cp38-cp38-linux_x86_64.whl

# Others.
# RUN cd /root \
#     && mkdir -p ocrtoc_ws/src \
#     && cd ocrtoc_ws \
#     && source /opt/ros/noetic/setup.bash \
#     && catkin_make -j8 \
#     && echo "source /root/ocrtoc_ws/devel/setup.bash" >> ~/.bashrc \
#     && rm -rf /var/lib/apt/lists/*

RUN ln -s /usr/bin/python3 /usr/bin/python
