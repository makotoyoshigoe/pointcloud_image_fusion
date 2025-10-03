// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_image_fusion/fusion_node.hpp"
#include "pointcloud_image_fusion/sensor_utils.hpp"
#include <chrono>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/create_timer_ros.h>
#include <image_transport/image_transport.hpp>

#include <cv_bridge/cv_bridge.h>

namespace lc_fusion {

template<typename T_p>
FusionNode<T_p>::FusionNode()
    : Node("lc_fusion")
    , sync_(approximate_policy_(10),
          sub_point_cloud_, sub_image_, sub_camera_info_)
    , init_tf_(false)
{
    declare_param();
	init_param();
    init_vg_filter();
    init_pubsub();
    coloring_.reset(new Coloring(rm_outrange_));
}

template<typename T_p>
FusionNode<T_p>::~FusionNode(void) { }

template<typename T_p>
void FusionNode<T_p>::init_pubsub(void)
{
    pub_color_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "colored_cloud", rclcpp::QoS(10));
    pub_depth_image_ = create_publisher<sensor_msgs::msg::Image>(
        "depth/ref_cloud", rclcpp::QoS(10));
    sub_point_cloud_.subscribe(this, "pointcloud");
    sub_image_.subscribe(this, "image_raw");
    sub_camera_info_.subscribe(this, "camera_info");
    sync_.registerCallback(&FusionNode<T_p>::fusion_cb, this);
}

template<typename T_p>
void FusionNode<T_p>::declare_param(void)
{
	this->declare_parameter("remove_outrange", true);
	this->declare_parameter("remove_outlier", false);
    this->declare_parameter("leaf_size", 0.05);
}

template<typename T_p>
void FusionNode<T_p>::init_param(void)
{
	rm_outrange_ = this->get_parameter("remove_outrange").as_bool();
	rm_outlier_  = this->get_parameter("remove_outlier").as_bool();
}

template<typename T_p>
void FusionNode<T_p>::init_vg_filter(void)
{
    double leaf_size = this->get_parameter("leaf_size").as_double();
    voxel_grid_filter_.reset(new pcl::VoxelGrid<pcl::PointXYZRGB>());
    voxel_grid_filter_->setLeafSize(leaf_size, leaf_size, leaf_size);
}

template<typename T_p>
void FusionNode<T_p>::init_tf(void)
{
    tf_buffer_.reset();
    tf_listener_.reset();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface(),
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false));
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    init_tf_ = true;
}

template<typename T_p>
bool FusionNode<T_p>::get_pose_from_lidar_to_camera(
    std::string lidar_frame_id, std::string camera_frame_id,
    tf2::Transform& tf)
{
    rclcpp::Time time = rclcpp::Time(0);
    geometry_msgs::msg::TransformStamped tf_tmp;

    try {
        tf_tmp = tf_buffer_->lookupTransform(
            camera_frame_id, lidar_frame_id, time, rclcpp::Duration::from_seconds(4.0));
        tf2::fromMsg(tf_tmp.transform, tf);
    } catch (tf2::TransformException& e) {
        RCLCPP_WARN(
            this->get_logger(), "Failed to compute camera pose(%s)", e.what());
        return false;
    }
    return true;
}

template<typename T_p>
void FusionNode<T_p>::fusion_cb(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg,
    sensor_msgs::msg::Image::ConstSharedPtr img_msg,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg)
{
    // Initialize TF
    if (!init_tf_)
        init_tf();

    // Get pose from lidar to camera
    tf2::Transform tf_lidar_camera;
    bool get_tf = get_pose_from_lidar_to_camera(
        cloud_msg->header.frame_id, img_msg->header.frame_id, tf_lidar_camera);
    if (!get_tf)
        return;
    // RCLCPP_INFO(this->get_logger(), "GET TF");

    // Transform cloud msg from ROS to PCL
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    transform_cloud<T_p>(cloud_msg, trans_cloud, tf_lidar_camera);
    // RCLCPP_INFO(this->get_logger(), "TRANSFORM CLOUD");

    // Downsampling by VoxelGrid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    voxel_grid_filter_->setInputCloud(trans_cloud);
    voxel_grid_filter_->filter(*sampled_cloud);

    // Transform img msg from ROS to CV
    cv::Mat rgb_image;
    bool trans_img = transform_image(
        img_msg, rgb_image);
    if (!trans_img)
        return;
    // RCLCPP_INFO(this->get_logger(), "TRANSFORM IMAGE");

    // Set PinholeCameraModel
    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(info_msg);

    // Sensor fusion
	// If rm_outrange is true, remove outrange of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    coloring_->coloring(sampled_cloud, colored_cloud, cam_model, rgb_image);
    // RCLCPP_INFO(this->get_logger(), "SENSOR FUSION");

    // Publish point cloud
    publish_color_cloud(
        colored_cloud, tf_lidar_camera, cloud_msg->header.frame_id);
    // RCLCPP_INFO(this->get_logger(), "PUBLISH CLOUD");
}

template<typename T_p>
void FusionNode<T_p>::publish_color_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    tf2::Transform& tf,
    std::string lidar_frame_id)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*colored_cloud, *output_cloud, tf.inverse());
	// RCLCPP_INFO(get_logger(), "TRANSFORM CLOUD");

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header.frame_id = lidar_frame_id;
    output_msg.header.stamp = now();
    pub_color_pc2_->publish(output_msg);
}

}
