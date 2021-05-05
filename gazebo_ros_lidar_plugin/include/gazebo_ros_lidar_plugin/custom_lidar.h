/*
 * Desc: LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@robotion.ai
 * Date: 5 September 2020
 * 
 * Copyright 2020 Robotion-AI. All rights reserved.
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

#include "dataContainers.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <algorithm>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class CustomLidar
{
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
    void construct_lidar_frame(std::vector<float> ranges, cv::Mat cam_feed_, PointCloud* pcl_msg);

    /// \brief color a point cloud
    pcl::PointXYZRGB color_point(cv::Mat cam_feed, pcl::PointXYZ pt);

    /// \brief mounting position point compensation
    pcl::PointXYZ mount_position_compensation(pcl::PointXYZ pt);

private:
    std::vector<double> arange(double minVal, double maxVal, double resolution);
};
#endif