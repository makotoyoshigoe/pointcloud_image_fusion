#!/bin/bash

# CLI11
cd $HOME
git clone https://github.com/CLIUtils/CLI11.git -b main
cd CLI11/
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install

# realsense packages
cd $HOME
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-realsense2-camera ros-${ROS_DISTRO}-realsense2-camera-msgs ros-${ROS_DISTRO}-realsense2-description

