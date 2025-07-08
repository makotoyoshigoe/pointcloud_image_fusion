// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace lc_fusion {
    bool transform_image(
        sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        cv::Mat& rgb_image);
    template<typename T_p>
    void transform_cloud(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& trans_cloud,
        tf2::Transform& tf);
}
#include "pointcloud_image_fusion/sensor_utils.ipp"
