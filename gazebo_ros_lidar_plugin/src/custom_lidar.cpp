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

#include "custom_lidar.h"

CustomLidar::CustomLidar()
{
    this->LP.parameters_initialized = false;
}
CustomLidar::~CustomLidar()
{
}
void CustomLidar::init_lidar_params()
{
    this->LP.parameters_initialized = true;
    this->LP.azimuthVec = arange(this->LP.min_horizontal_angle,
                                 this->LP.max_horizontal_angle +
                                     this->LP.horizontal_step_angle,
                                 this->LP.horizontal_step_angle);
    this->LP.elevationVec = arange(this->LP.min_vertical_angle,
                                   this->LP.max_vertical_angle +
                                       this->LP.vertical_step_angle,
                                   this->LP.vertical_step_angle);
}

void CustomLidar::construct_lidar_frame(std::vector<float> ranges,
                                        cv::Mat cam_feed_, PointCloud *pcl_msg)
{
    for (int pointIdx = 0; pointIdx < ranges.size(); pointIdx++)
    {
        int aximuthIdx = pointIdx % static_cast<int>(this->LP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->LP.horizontal_points_count);
        if (std::numeric_limits<float>::infinity() == ranges[pointIdx])
            ranges[pointIdx] = 0;

        // Polar to cartisian coordinates transformation.
        pcl::PointXYZ pt;
        pt.x = ranges[pointIdx] * cos(this->LP.azimuthVec[aximuthIdx]) *
               cos(this->LP.elevationVec[elevationIdx]);
        pt.y = ranges[pointIdx] * sin(this->LP.azimuthVec[aximuthIdx]) *
               cos(this->LP.elevationVec[elevationIdx]);
        pt.z = ranges[pointIdx] * sin(this->LP.elevationVec[elevationIdx]);
        pcl::PointXYZRGB colored_pt = color_point(cam_feed_, pt);
        pt = mount_position_compensation(pt);
        colored_pt.x = pt.x;
        colored_pt.y = pt.y;
        colored_pt.z = pt.z;
        pcl_msg->points.push_back(colored_pt);
    }
}

pcl::PointXYZRGB CustomLidar::color_point(cv::Mat cam_feed, pcl::PointXYZ pt)
{
    std::uint8_t R = 0, G = 0, B = 0;
    if (this->LP.enable_fusion)
    {
        // Fuse camera with lidar point cloud.
        float focalLength = (cam_feed.cols / 2) / tan(this->LP.cam_fov / 2);
        float cx = cam_feed.cols / 2;
        float cy = cam_feed.rows / 2;

        // Camera x-axis is LiDAR -1*y-axis, camera y-axis is LiDAR -1*z-axis, and
        // camera z-axis(deoth) is LiDAR x_axis.
        float X_cam = -pt.y;
        float Y_cam = -pt.z;
        float Z_cam = pt.x;
        X_cam = static_cast<int>(cx + X_cam * focalLength / Z_cam);
        Y_cam = static_cast<int>(cy + Y_cam * focalLength / Z_cam);
        if (Y_cam < cy * 2 - 1 && Y_cam > 0 &&
            X_cam < cx * 2 - 1 && X_cam > 0 &&
            Z_cam > 0)
        {
            B = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[0];
            G = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[1];
            R = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[2];
        }
        pcl::PointXYZRGB colored_pt = pcl::PointXYZRGB(R, G, B);
        return colored_pt;
    }
    else
    {
        pcl::PointXYZRGB colored_pt = pcl::PointXYZRGB(R, G, B);
        return colored_pt;
    }
}
pcl::PointXYZ CustomLidar::mount_position_compensation(pcl::PointXYZ point)
{
    pcl::PointXYZ pt;
    pt.x = point.x * cos(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
           point.y * (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) -
                      sin(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
           point.z * (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) +
                      sin(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
           this->LP.mp.x;
    pt.y = point.x * sin(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
           point.y * (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
                      cos(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
           point.z * (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) -
                      cos(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
           this->LP.mp.y;
    pt.z = point.x * (-sin(this->LP.mp.pitch)) +
           point.y * cos(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
           point.z * cos(this->LP.mp.pitch) * cos(this->LP.mp.roll) + this->LP.mp.z;
    return pt;
}
std::vector<double> CustomLidar::arange(double minVal,
                                        double maxVal,
                                        double resolution)
{
    // We get a vector varying from min val to max val with a step resolution.
    std::vector<double> vec;
    double minimum = std::min(minVal, maxVal);
    double maximum = std::max(minVal, maxVal);
    for (double i = minimum; i < maximum; i += resolution)
    {
        vec.push_back(i);
    }
    return vec;
}
