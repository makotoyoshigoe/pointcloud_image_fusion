# pointcloud_image_fusion [![build-test](https://github.com/makotoyoshigoe/pointcloud_image_fusion/actions/workflows/build-test.yaml/badge.svg)](https://github.com/makotoyoshigoe/pointcloud_image_fusion/actions/workflows/build-test.yaml)
このパッケージは、3D LiDARや深度カメラなどが出力する3次元点群とRGBカメラなどが出力するRGB画像を合成し、3次元の色付き点群を出力します。

# Development environment
- Ubuntu 22.04.4 LTS
- ROS 2 Humble
- Open CV 4.5.4
- PCL 1.12.1

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

## RealSense related packages (Opttion)
```
sudo apt update
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-camera-msgs ros-humble-realsense2-description
```

## Preparation
入力の3次元点群の型を確認してください。現在の実装では、3次元の位置情報（xyz）と、それに反射強度を加えた情報（xyzi）をサポートしています。この2つの型であれば今のままでプログラムを動作させることができますので、次のステップに進んでください。もし、この2つ以外の型であれば、以下のようにして対応してください。
1.  Point Cloud Library (PCL) の型で一致しているものを見つける。
2.  ```pointcloud_image_fusion/src/main.cpp```を書き換え、型を対応させる。
3.  ```pointcloud_image_fusion/launch/```にあるいずれかのlaunchファイルをコピーして、必要な箇所を書き換える
### Example: 入力点群の型がXYZN (x, y, z, normal_x, normal_y, normal_z) の場合
1. XYZN -> pcl::PointXYZNがある
2. main.cppの該当箇所を編集
```cpp
...
} else if (pc_type == "xyzi") {
    return std::make_shared<lc_fusion::FusionNode<pcl::PointXYZI>>();
} 
↓を追加                 ↓を適当な名前に設定 
else if (pc_type == "xyzn") {
    return std::make_shared<lc_fusion::FusionNode<pcl::PointXYZN>>();
} else { 
...
```
3. launchファイルをコピーして、`--pc-type`の引数の値を`main.cpp`で書いたものと一致させる
```py
...
Node(
    package='pointcloud_image_fusion',
    executable='lc_fusion',
    name='lc_fusion_node',
    output='screen',
                              ↓を書き換える
    arguments=['--pc-type', 'xyzn', '--ros-args', '--log-level', 'warn'],
...
```

## Build
```
cd ~/ros2_ws && colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```
# Usage
## Run
- Ex: 3D LiDAR (Livox Mid360) & RGBDカメラ (Realsense D435)
※launchファイルの動作未確認
```
ros2 launch pointcloud_image_fusion livox_realsense.launch.py
```
- Ex: RGBDカメラのみ (Realsense D435)
```
ros2 launch pointcloud_image_fusion realsense_only.launch.py
```

# Input
|Topic name|Type|Description|
|---|---|---|
|/pointcloud|sensor_msgs::msg::PointCloud2|入力3次元点群|
|/image_raw|sensor_msgs::msg::Image|入力RGB画像|
|/camera_info|sensor_msgs::msg::CameraInfo|カメラの内部パラメータ|
|/tf, /tf_static|tf2::Transform|センサの位置関係|

# Output
|Topic name|Type|Description|
|---|---|---|
|/colored_cloud|sensor_msgs::msg::PointCloud2|出力色付き3次元点群|

# Parameter
|Name|Type|Default|Description|
|---|---|---|---|
|leaf_size|Double|0.05|VoxelGridフィルターのVoxelの1辺[m]
|remove_outrange|Bool|true|色がつかない点群を削除する|
|remove_outlier|Bool|false|外れ値の点群を削除（未実装）|

# Reference
- [velocyne_camera_calibration](https://github.com/Sadaku1993/velodyne_camera_calibration/tree/master)
