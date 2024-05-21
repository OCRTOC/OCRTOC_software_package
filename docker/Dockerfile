from nvidia/cuda:12.0.0-devel-ubuntu22.04

ENV TZ=Asia/Shanghai
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute


# Install Ubuntu tools.
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone \
    && apt-get update \
    && apt-get install -y build-essential git libssl-dev \
        lsb-release iputils-ping manpages-dev net-tools \
        python3-pip python3-tk \
        psmisc software-properties-common tree vim wget zip \
    # Install gl/x package for visualization inside docker
    && apt-get install -y libglvnd0 libgl1 libglx0 libegl1 libxext6 libx11-6 


# Install mujoco, gym and their dependency
RUN pip3 install numpy==1.26.4 transforms3d==0.4.1 mujoco==3.1.3 scipy==1.12.0 gymnasium==0.29.1  joblib==1.3.2 matplotlib==3.9.0 kinpy==0.4.2  


