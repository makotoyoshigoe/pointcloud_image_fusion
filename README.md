# pointcloud_image_fusion [![build-test](https://github.com/makotoyoshigoe/pointcloud_image_fusion/actions/workflows/build-test.yaml/badge.svg)](https://github.com/makotoyoshigoe/pointcloud_image_fusion/actions/workflows/build-test.yaml)

# Installation
## CLI11
```
git clone https://github.com/CLIUtils/CLI11.git -b main
cd CLI11/
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```
## RealSense related packages
```
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-realsense2-description
```

# Reference
- [velocyne_camera_calibration](https://github.com/Sadaku1993/velodyne_camera_calibration/tree/master)

