from nvidia/cudagl:11.2.1-devel-ubuntu18.04

ENV TZ=Asia/Shanghai
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

# Install Ubuntu tools.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && rm /etc/apt/sources.list.d/cuda.list \
    && rm /etc/apt/sources.list.d/nvidia-ml.list \
    && apt-get update \
    && apt-get install -y build-essential git libssl-dev \
        lsb-release iputils-ping manpages-dev net-tools \
        python-pip python3-pip python3-tk \
        psmisc software-properties-common tree vim wget \
    # Install gl/x package for visualization inside docker
    && apt-get install -y libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 \
    # Install gcc9
    && add-apt-repository -y ppa:ubuntu-toolchain-r/test && apt-get update \
    && apt install -y gcc-9 g++-9 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 70 \
        --slave /usr/bin/g++ g++ /usr/bin/g++-7 \
        --slave /usr/bin/gcov gcov /usr/bin/gcov-7 \
    && update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 90 \
        --slave /usr/bin/g++ g++ /usr/bin/g++-9 \
        --slave /usr/bin/gcov gcov /usr/bin/gcov-9 \
    # Install cmake 3.16.3
    && cd /root && wget -q https://ocrtoc-public.oss-cn-hangzhou.aliyuncs.com/docker/cmake-3.16.3.tar.gz \
    && tar -xf cmake-3.16.3.tar.gz && cd cmake-3.16.3 \
    && ./bootstrap && make -j4 && make install \
    && cd /root && rm -rf cmake-3.16.3 cmake-3.16.3.tar.gz

# Install ROS.
SHELL ["/bin/bash", "-c"]
RUN sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ `lsb_release -cs` main" > /etc/apt/sources.list.d/ros-latest.list' \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt update \
    && apt install -y ros-melodic-desktop-full \
    && echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc \
    && source ~/.bashrc \
    && apt install -y python-rosdep \
    && echo "151.101.84.133  raw.githubusercontent.com" >> /etc/hosts \
    && rosdep init \
    && rosdep update

# Install ROS dependencies.
RUN pip3 install catkin_pkg numpy transforms3d rospkg \
    && apt-get update && apt-get install -y alien \
    && apt-get install -y ros-melodic-ddynamic-reconfigure \
                          ros-melodic-effort-controllers \
                          ros-melodic-franka-ros \
                          ros-melodic-gripper-action-controller \
                          ros-melodic-joint-state-controller \
                          ros-melodic-joint-trajectory-controller \
                          ros-melodic-industrial-robot-status-interface \
                          ros-melodic-libfranka \
                          ros-melodic-moveit \
                          ros-melodic-position-controllers \
                          ros-melodic-realsense2-camera \
                          ros-melodic-ros-control \
                          ros-melodic-ros-controllers \
                          ros-melodic-ros-numpy \
                          ros-melodic-socketcan-interface \
                          ros-melodic-soem \
                          ros-melodic-ur-msgs \
                          ros-melodic-visp-hand2eye-calibration

# Install drivers.
RUN curl -sSL https://packages.microsoft.com/keys/microsoft.asc | apt-key add - \
    && apt-add-repository -y https://packages.microsoft.com/ubuntu/18.04/prod \
    && apt-get update \
    && echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections \
    && echo 'libk4a1.4 libk4a1.4/accept-eula boolean true' | debconf-set-selections \
    && apt install -y k4a-tools libk4a1.4-dev \
    && apt-get install python-pymodbus socat \
    && apt-key adv --keyserver keys.gnupg.net \
        --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv \
        --keyserver hkp://keyserver.ubuntu.com:80 \
        --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

RUN git clone https://github.com/IntelRealSense/librealsense.git --recursive \
    && cd librealsense \
    && apt update \
    && apt -y install git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at\
    && mkdir build \
    && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=release \
    && make -j8 \
    && make install

# RUN add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u \
#     && apt update \
#     && apt-get -y install librealsense2-dkms librealsense2-utils \
#         librealsense2-dev librealsense2-dbg

# Install Pybullet.
RUN python3 -m pip install pybullet==3.0 pyquaternion -i https://mirrors.aliyun.com/pypi/simple \
    && python -m pip install pip --upgrade -i https://mirrors.aliyun.com/pypi/simple \
    && python -m pip install pybullet==3.0 numpy --upgrade -i https://mirrors.aliyun.com/pypi/simple \
    && python -m pip install scipy open3d \
    && apt update \
    && apt -y install htop imagemagick

# Install Sapien dependences.
RUN wget -qO - https://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - \
    && wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.2.170-bionic.list https://packages.lunarg.com/vulkan/1.2.170/lunarg-vulkan-1.2.170-bionic.list \
    && apt-get update && apt-get install -y vulkan-sdk libglew-dev \
    && cd /root && git clone https://github.com/assimp/assimp \
    && cd assimp && git checkout v5.0.1 && mkdir build && cd build \
    && cmake .. && make -j 8 && make install \
    && cd /root && rm -rf assimp \
    && git clone https://github.com/g-truc/glm \
    && cd glm && git checkout 0.9.9.8 && cp -r glm /usr/local/include \
    && cd /root && rm -rf glm \
    && mkdir -p /workspace/sapien_pack && cd /workspace/sapien_pack\
    && git clone -b v1.8.5 https://github.com/gabime/spdlog \
    && git clone -b 3.3.3 https://github.com/glfw/glfw \
    && cd /workspace/sapien_pack/glfw \
    && mkdir build && cd build && cmake .. && make install -j8\
    && cd /workspace/sapien_pack/spdlog \
    && mkdir build && cd build && cmake .. && make install -j8

# For ocrtoc_common.
RUN cd /workspace && mkdir -p geometry2_ws/src && cd geometry2_ws/src \
    && git clone -b melodic-devel https://github.com/ros/geometry2.git \
    && cd .. && source /opt/ros/melodic/setup.bash \
    && pip install transforms3d \
    && pip3 install empy pyyaml virtualenv \
    && virtualenv -p /usr/bin/python3 venv \
    && source venv/bin/activate \
    && catkin_make --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
        -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
    && cd devel/lib/python3/dist-packages \
    && cp -r * /usr/local/lib/python3.6/dist-packages/

# For perception.
RUN python3 -m pip install pip --upgrade \
    && python3 -m pip install scikit-build Cython \
    && python3 -m pip install matplotlib graspnetAPI \
        open3d==0.12.0.0 open3d-plus tensorboard==2.3 \
        pyrealsense2 pyrender trimesh plyfile scikit-learn pytz \
    && python3 -m pip install imageio==2.5.0 pypng==0.0.19 PyOpenGL==3.1.0 \
        triangle==20190115.2 glumpy==1.1.0 \
        Pillow==7.2.0 rtree -i https://mirrors.aliyun.com/pypi/simple \
    && python3 -m pip install torch==1.8.2+cu111 torchvision==0.9.2+cu111 \
    -f https://download.pytorch.org/whl/lts/1.8/torch_lts.html

# For perception.
RUN apt -y install libboost-program-options-dev \
    libboost-filesystem-dev libboost-graph-dev libboost-system-dev \
    libboost-test-dev libeigen3-dev libsuitesparse-dev libfreeimage-dev \
    libgoogle-glog-dev libgflags-dev qtbase5-dev libqt5opengl5-dev \
    libcgal-dev libcgal-qt5-dev libatlas-base-dev libsuitesparse-dev \
    && git clone https://ceres-solver.googlesource.com/ceres-solver \
    && cd ceres-solver && git checkout $(git describe --tags) \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && make -j8 && make install \
    && cd ../.. && rm -rf ceres-solver \
    && git clone https://github.com/colmap/colmap \
    && cd colmap && git checkout dev \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_C_COMPILER=/usr/bin/gcc-7 && make -j8 && make install \
    && cd ../.. && rm -rf colmap \
    && git clone --recursive https://github.com/mihaidusmanu/pycolmap.git \
    && cd pycolmap && python3 -m pip install . \
    && cd .. && rm -rf pycolmap

# Build Sapien.
RUN cd /workspace/sapien_pack \
    && wget -q https://ocrtoc-public.oss-cn-hangzhou.aliyuncs.com/docker/sapien_ros_docker_build_src_2.0.zip \
    && unzip sapien_ros_docker_build_src_2.0.zip \
    && mv sapien_ros_docker_build_src_2.0/* . \
    && rm -rf sapien_ros_docker_build_src_2.0* \
    && mkdir -p /usr/share/vulkan/icd.d \
    && cp nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json \
    && source /opt/ros/melodic/setup.bash \
    && cd sapien_ros && python3 setup.py bdist_wheel \
    && pip3 install dist/sapien_ros1-1.0.0rc2-cp36-cp36m-linux_x86_64.whl

# Others.
RUN cd /root \
    && mkdir -p ocrtoc_ws/src \
    && cd ocrtoc_ws \
    && source /opt/ros/melodic/setup.bash \
    && catkin_make -j8\
    && echo "source /root/ocrtoc_ws/devel/setup.bash" >> ~/.bashrc \
    && rm -rf /var/lib/apt/lists/*
