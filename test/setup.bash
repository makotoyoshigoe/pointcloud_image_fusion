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
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-realsense2-description

