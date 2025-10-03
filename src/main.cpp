// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_image_fusion/fusion_node.hpp"
#include <CLI/CLI.hpp>

std::shared_ptr<rclcpp::Node> create_node_by_name(const std::string& pc_type) {
    if (pc_type == "xyz") {
        return std::make_shared<lc_fusion::FusionNode<pcl::PointXYZ>>();
    } else if (pc_type == "xyzi") {
        return std::make_shared<lc_fusion::FusionNode<pcl::PointXYZI>>();
    } else {
        throw std::runtime_error("Unknown point type: " + pc_type);
    }
}

int main(int argc, char** argv)
{
	std::vector<std::string> cli_args = rclcpp::remove_ros_arguments(argc, argv);

    // vector<string> → vector<char*> に変換
    std::vector<char*> cli_argv;
    for (auto& s : cli_args) {
        cli_argv.push_back(const_cast<char*>(s.c_str()));
    }

    // CLI11 パース
    CLI::App app{"Input pointcloud type"};
    std::string pc_type;
    app.add_option("--pc-type", pc_type, "input pointcloud type")->required();
    CLI11_PARSE(app, static_cast<int>(cli_argv.size()), cli_argv.data());

    // ROS初期化（--ros-args などが消費される）
    rclcpp::init(argc, argv);

    // ノード生成・起動
    auto node = create_node_by_name(pc_type);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;



	// Set pointcloud type
	//auto cli_args = rclcpp::remove_ros_arguments(argc, argv);
	//CLI::App app{"Input pointcloud type"};
	//std::string pc_type;
	//app.add_option("--pc-type", pc_type, "input pointcloud type")->required();
	////CLI11_PARSE(app, cli_args.argc(), cli_args.argv());
	//CLI11_PARSE(app, static_cast<int>(cli_args.size()), cli_args.data());

	//rclcpp::init(argc, argv);
    //auto node = create_node_by_name(pc_type);
    //rclcpp::spin(node);
    //rclcpp::shutdown();
    //return 0;
}

