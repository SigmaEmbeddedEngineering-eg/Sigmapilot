/*
 * Desc: LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 5 September 2020
 *
 * Copyright 2020 sigma embedded engineering-se. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef CUSTOM_LIDAR_HH
#define CUSTOM_LIDAR_HH

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "dataContainers.h"

class CustomLidar {
   public:
    /// \brief Constructor
    CustomLidar();

    /// \brief Destructor
    ~CustomLidar();

    /// \brief initialize from first scan
    void init_lidar_params();

    /// \brief Lidar parameters data container
    LidarParameters LP;

    /// \brief create a frame
    ErrorCode construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const& cam_feed_, PointCloud& pcl_msg);

    /// \brief color a point cloud
    ErrorCode color_point(cv::Mat const& cam_feed, pcl::PointXYZ const& pt, pcl::PointXYZRGB& colored_pt);

    /// \brief mounting position point compensation
    ErrorCode mount_position_compensation(pcl::PointXYZ const& point, pcl::PointXYZ& calibrated_pt);

   private:
    ErrorCode arange(double const& minVal, double const& maxVal, double const& resolution, std::vector<double>& vec);
};
#endif