FROM osrf/ros:humble-desktop


# Build tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    ninja-build \
    pkg-config \
    libevdev-dev \
    python3-colcon-common-extensions \
    python3-scipy \
    python3-pip \
    python3-venv \
    xauth \
    && rm -rf /var/lib/apt/lists/*

# ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-costmap-2d \
    ros-humble-slam-toolbox \
    ros-humble-tf2-ros \
    ros-humble-rviz2 \
    ros-humble-pcl-conversions \
    ros-humble-laser-geometry \
    ros-humble-image-geometry \
    ros-humble-rtabmap-ros \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-camera-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python requirements
COPY requirements.txt /tmp/requirements.txt
RUN pip install -r /tmp/requirements.txt
