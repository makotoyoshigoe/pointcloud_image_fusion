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

#include <pcl/filters/voxel_grid.h>

#include "pointcloud_image_fusion/coloring.hpp"

namespace lc_fusion {
template<typename T_p>
class FusionNode : public rclcpp::Node {
public:
    FusionNode(void);
    ~FusionNode();
    void init_pubsub(void);
	void init_param(void);
    void init_vg_filter(void);
    void fusion_cb(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
        sensor_msgs::msg::Image::ConstSharedPtr img_msg,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg);
    void init_tf(void);
    bool get_pose_from_lidar_to_camera(
        std::string lidar_frame_id, std::string camera_frame_id,
        tf2::Transform& tf);
    void publish_color_cloud(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        tf2::Transform& tf,
        std::string lidar_frame_id);

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_color_pc2_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_depth_image_;
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

    std::unique_ptr<Coloring> coloring_;
    bool init_tf_;
	bool rm_outrange_, rm_outlier_;

    pcl::VoxelGrid<pcl::PointXYZRGB>::Ptr voxel_grid_filter_;
};

}

#include "pointcloud_image_fusion/fusion_node.ipp"
