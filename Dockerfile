FROM osrf/ros:humble-desktop

WORKDIR /

# Python
RUN apt -y update \
    && apt install -y python3-pip \
    && pip install --no-cache-dir --upgrade pip

# Python packages
# RUN pip install setuptools==58.2.0 \
#     && pip install numpy==1.24.4 \
#     && pip install --ignore-installed sympy==1.10 \
#     && pip install --no-cache-dir torch torchvision torchaudio \
#     && pip install --no-cache-dir ultralytics \
#     && pip install --no-cache-dir dill \
#     && pip install --no-cache-dir scipy \
#     && pip install --no-cache-dir ros2-numpy \
#     && pip install --no-cache-dir squaternion \
#     && pip install --no-cache-dir quadprog==0.1.12

# Gazebo Fortress
RUN apt -y update \
    && apt install -y ros-${ROS_DISTRO}-ros-gz \
    && apt install -y ros-humble-turtlebot3* \
    && apt -y update

# Navigation2
RUN apt -y update \
    && apt install -y ros-humble-navigation2 ros-humble-nav2-bringup \
    && apt install -y ros-humble-ros-ign-bridge

# Cpp tools and libraries
RUN apt -y update \
    && apt install -y vim \
    && apt install -y cmake \
    && apt install -y --no-install-recommends build-essential \
    && apt install -y libeigen3-dev libboost-all-dev libtbb-dev

COPY dep /dep

# Gtsam library
WORKDIR /dep/gtsam-4.2.0-ros
RUN mkdir build \
    && cd build \
    && cmake .. \
    && make install \
    && ldconfig

# WORKDIR /dep/BehaviorTree.CPP
# RUN mkdir build \
#     && cd build \
#     && cmake .. \
#     && make install \
#     && ldconfig

COPY workspace /workspace

# Configuration .bashrc

RUN echo "# Set default XDG_RUNTIME_DIR" >> /root/.bashrc \
    && echo "export XDG_RUNTIME_DIR=/tmp/runtime-root" >> /root/.bashrc

RUN echo "# Avoid graphic issues using DRI2 instead of DRI3 with Intel Iris" >> /root/.bashrc \
    && echo "export LIBGL_DRI3_DISABLE=1" >> /root/.bashrc

RUN echo "# Source ROS2 Humble environment" >> /root/.bashrc \
    && echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

RUN echo "# Change dir to ros2_ws when container is started" >> /root/.bashrc \
    && echo "cd /workspace/" >> /root/.bashrc
    
WORKDIR /workspace
