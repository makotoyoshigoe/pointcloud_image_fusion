// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_image_fusion/sensor_utils.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

namespace lc_fusion {

bool transform_image(
    sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
    cv::Mat& rgb_image)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try {
        cv_img_ptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(rclcpp::get_logger("lc_fusion"), "cv_bridge exception: %s", e.what());
        return false;
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(img_msg)->image;

    cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);
    return true;
}

template <typename T_p>
void transform_cloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& trans_cloud,
    tf2::Transform& tf)
{
    //pcl::PointCloud<pcl::PointXYZI>::Ptr sensor_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    typename pcl::PointCloud<T_p>::Ptr sensor_cloud(new pcl::PointCloud<T_p>);
    pcl::fromROSMsg(*cloud_msg, *sensor_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*sensor_cloud, *cloud);

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
}

}
