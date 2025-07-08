// SPDX-FileCopyrightText: 2025 Makoto Yoshigoe myoshigo0127@gmail.com
// SPDX-License-Identifier: Apache-2.0

#include "pointcloud_image_fusion/coloring.hpp"

namespace lc_fusion {
Coloring::Coloring(bool rm_outrange)
: rm_outrange_(rm_outrange){ }

Coloring::~Coloring(void) { }

void Coloring::coloring(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& base_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud,
    image_geometry::PinholeCameraModel& cam_model,
    cv::Mat& rgb_image)
{
    pcl::PointCloud<pcl::PointXYZRGB>::iterator pt;
    for (pt = base_cloud->points.begin(); pt < base_cloud->points.end(); pt++) {
        if ((*pt).z < 0) {
			//if(rm_outrange_) colored_cloud->erase(pt);
			if(rm_outrange_) continue;
            set_cloud_color(pt, 255);
			colored_cloud->points.push_back((*pt));
        } else {
            cv::Point3d pt_cv((*pt).x, (*pt).y, (*pt).z);
            cv::Point2d uv = cam_model.project3dToPixel(pt_cv);
            if (uv.x > 0 && uv.x < rgb_image.cols && uv.y > 0 && uv.y < rgb_image.rows) {
                // Coloring PointCloud
                set_cloud_color(pt, rgb_image.at<cv::Vec3b>(uv));
				colored_cloud->points.push_back((*pt));
            } else {
				if(rm_outrange_) continue;
            	set_cloud_color(pt, 255);
				colored_cloud->points.push_back((*pt));
			}
        }
    }
	colored_cloud->width = colored_cloud->points.size();
	colored_cloud->height = 1;
	colored_cloud->is_dense = false;
}

void Coloring::set_cloud_color(
    pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
    cv::Vec3b& cv_vec)
{
    (*pt).b = cv_vec[0];
    (*pt).g = cv_vec[1];
    (*pt).r = cv_vec[2];
}

void Coloring::set_cloud_color(
    pcl::PointCloud<pcl::PointXYZRGB>::iterator& pt,
    int c)
{
    (*pt).b = c;
    (*pt).g = c;
    (*pt).r = c;
}

}
