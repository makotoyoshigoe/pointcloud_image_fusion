// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "lidar_camera_fusion/fusion_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<lc_fusion::FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
