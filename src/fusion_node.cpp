// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "lidar_camera_fusion/fusion_node.hpp"

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

FusionNode::FusionNode()
    : Node("lc_fusion")
    , sync_(approximate_policy_(10),
          sub_point_cloud_, sub_image_, sub_camera_info_)
    , init_tf_(false)
{
    init_pubsub();
}

FusionNode::~FusionNode(void) { }

void FusionNode::init_pubsub(void)
{
    pub_color_pc2_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "colored_cloud", rclcpp::QoS(10));
    sub_point_cloud_.subscribe(this, "livox/lidar");
    sub_image_.subscribe(this, "camera/camera/color/image_raw");
    sub_camera_info_.subscribe(this, "camera/camera/color/camera_info");
    sync_.registerCallback(&FusionNode::fusion_cb, this);
}

void FusionNode::init_tf(void)
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

bool FusionNode::get_pose_from_lidar_to_camera(
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

void FusionNode::fusion_cb(
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
    transform_cloud(cloud_msg, trans_cloud, tf_lidar_camera);
    // RCLCPP_INFO(this->get_logger(), "TRANSFORM CLOUD");

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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud = std::move(trans_cloud);
    sensor_fusion(colored_cloud, cam_model, rgb_image);
    // RCLCPP_INFO(this->get_logger(), "SENSOR FUSION");

    // Publish point cloud
    publish_color_cloud(
        colored_cloud, tf_lidar_camera, cloud_msg->header.frame_id);
    // RCLCPP_INFO(this->get_logger(), "PUBLISH CLOUD");
}

void FusionNode::transform_cloud(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_msg,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& trans_cloud,
    tf2::Transform& tf)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr sensor_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*cloud_msg, *sensor_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*sensor_cloud, *cloud);

    pcl_ros::transformPointCloud(*cloud, *trans_cloud, tf);
}

bool FusionNode::transform_image(
    sensor_msgs::msg::Image::ConstSharedPtr& img_msg,
    cv::Mat& rgb_image)
{
    cv_bridge::CvImageConstPtr cv_img_ptr;
    try {
        cv_img_ptr = cv_bridge::toCvShare(img_msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
        return false;
    }

    cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
    cv_image = cv_bridge::toCvShare(img_msg)->image;

    cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);
    return true;
}

void FusionNode::sensor_fusion(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    image_geometry::PinholeCameraModel& cam_model,
    cv::Mat& rgb_image)
{
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt;
    for (pt = colored_cloud->points.begin(); pt < colored_cloud->points.end(); pt++) {
        if ((*pt).z < 0) {
            set_cloud_color(pt, 255);
            // RCLCPP_INFO(this->get_logger(), "z<0");
        } else {
            cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
            cv::Point2d uv = cam_model.project3dToPixel(pt_cv);
            // RCLCPP_INFO(this->get_logger(), "uv(%lf, %lf), (cols, rows)=(%d, %d)",
            //		uv.x, uv.y, rgb_image.cols, rgb_image.rows);

            if (uv.x > 0 && uv.x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows) {
                // Coloring PointCloud
                set_cloud_color(pt, rgb_image.at<cv::Vec3b>(uv));
                // Projection PointCloud
                // double range = sqrt( pow((*pt).x, 2.0) + pow((*pt).y, 2.0) + pow((*pt).z, 2.0));
                // COLOUR c = GetColour(int(range/20*255.0), 0, 255);
                // cv::circle(
                // 		projection_image, uv, 1,
                // 		cv::Scalar(int(255*c.b),int(255*c.g),int(255*c.r)), -1);
            } else {
                set_cloud_color(pt, 255);
                //	RCLCPP_INFO(this->get_logger(), "out of range");
            }
        }
    }
}

void FusionNode::set_cloud_color(
    pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
    cv::Vec3b& cv_vec)
{
    (*pt).b = cv_vec[0];
    (*pt).g = cv_vec[1];
    (*pt).r = cv_vec[2];
}

void FusionNode::set_cloud_color(
    pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
    int c)
{
    (*pt).b = c;
    (*pt).g = c;
    (*pt).r = c;
}

void FusionNode::publish_color_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    tf2::Transform& tf,
    std::string lidar_frame_id)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl_ros::transformPointCloud(*colored_cloud, *output_cloud, tf.inverse());

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*output_cloud, output_msg);
    output_msg.header.frame_id = lidar_frame_id;
    output_msg.header.stamp = now();
    pub_color_pc2_->publish(output_msg);
}

}
