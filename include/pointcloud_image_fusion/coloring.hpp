// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>

namespace lc_fusion {
class Coloring {
public:
    Coloring(bool rm_outrange);
    ~Coloring(void);
    void coloring(
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& base_cloud,
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
        image_geometry::PinholeCameraModel& cam_model,
        cv::Mat& rgb_image);
    void set_cloud_color(
        pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
        cv::Vec3b& cv_vec);
    void set_cloud_color(
        pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
        int c);
private:
	bool rm_outrange_;
};

}
