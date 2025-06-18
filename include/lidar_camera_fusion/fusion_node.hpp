// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>

namespace lc_fusion {
class FusionNode : public rclcpp::Node {
public:
    FusionNode(void);
    ~FusionNode(void);
    void init_pubsub(void);
    void fusion_cb(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
        sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
    void init_tf(void);
    bool get_pose_from_lidar_to_camera(
        std::string lidar_frame_id, std::string camera_frame_id,
        tf2::Transform& tf);
    void transform_cloud(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& trans_cloud,
        tf2::Transform& tf);
    bool transform_image(
        sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
        cv::Mat& rgb_image);
    void sensor_fusion(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        image_geometry::PinholeCameraModel& cam_model,
        cv::Mat& rgb_image);
    void set_cloud_color(
        pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
        cv::Vec3b& cv_vec);
    void set_cloud_color(
        pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
        int c);
    void publish_color_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        tf2::Transform& tf,
        std::string lidar_frame_id);

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_color_pc2_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_point_cloud_;
    message_filters::Subscriber<sensor_msgs::msg::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> sub_camera_info_;
    using approximate_policy_ = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>;
    message_filters::Synchronizer<approximate_policy_> sync_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool init_tf_;
};

}
