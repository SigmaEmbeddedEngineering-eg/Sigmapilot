/*
 * Desc: LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 5 September 2020
 *
 * Copyright 2020 SigmaEmbeddedEngineering. All rights reserved.
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

CustomLidar::CustomLidar() { this->LP.parameters_initialized = false; }
CustomLidar::~CustomLidar() {}
void CustomLidar::init_lidar_params() {
    this->LP.parameters_initialized = true;
    (void)arange(this->LP.min_horizontal_angle, this->LP.max_horizontal_angle + this->LP.horizontal_step_angle,
                 this->LP.horizontal_step_angle, this->LP.azimuthVec);
    (void)arange(this->LP.min_vertical_angle, this->LP.max_vertical_angle + this->LP.vertical_step_angle,
                 this->LP.vertical_step_angle, this->LP.elevationVec);
}

ErrorCode CustomLidar::construct_lidar_frame(std::vector<float> const& ranges, cv::Mat const& cam_feed_,
                                             PointCloud& pcl_msg) {
    if (ranges.empty()) {
        return ErrorCode::BadArgument;
    }
    for (int pointIdx = 0; pointIdx < ranges.size(); pointIdx++) {
        int aximuthIdx = pointIdx % static_cast<int>(this->LP.horizontal_points_count);
        int elevationIdx = pointIdx / static_cast<int>(this->LP.horizontal_points_count);
        float r{0};
        if (std::numeric_limits<float>::infinity() == ranges[pointIdx])
            r = 0;
        else
            r = ranges[pointIdx];

        // Polar to cartisian coordinates transformation.
        pcl::PointXYZ pt;
        pt.x = r * cos(this->LP.azimuthVec[aximuthIdx]) * cos(this->LP.elevationVec[elevationIdx]);
        pt.y = r * sin(this->LP.azimuthVec[aximuthIdx]) * cos(this->LP.elevationVec[elevationIdx]);
        pt.z = r * sin(this->LP.elevationVec[elevationIdx]);
        pcl::PointXYZRGB colored_pt;
        (void)color_point(cam_feed_, pt, colored_pt);
        pcl::PointXYZ calibrated_pt;
        (void)mount_position_compensation(pt, calibrated_pt);
        colored_pt.x = calibrated_pt.x;
        colored_pt.y = calibrated_pt.y;
        colored_pt.z = calibrated_pt.z;
        pcl_msg.points.push_back(colored_pt);
    }
    return ErrorCode::Success;
}

ErrorCode CustomLidar::color_point(cv::Mat const& cam_feed, pcl::PointXYZ const& pt, pcl::PointXYZRGB& colored_pt) {
    std::uint8_t R{0}, G{0}, B{0};
    if (this->LP.enable_fusion) {
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
        if (Y_cam < cy * 2 - 1 && Y_cam > 0 && X_cam < cx * 2 - 1 && X_cam > 0 && Z_cam > 0) {
            B = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[0];
            G = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[1];
            R = cam_feed.at<cv::Vec3b>(Y_cam, X_cam)[2];
        }
        colored_pt = pcl::PointXYZRGB(R, G, B);
        return ErrorCode::Success;
    } else {
        colored_pt = pcl::PointXYZRGB(R, G, B);
        return ErrorCode::Failed;
    }
}
ErrorCode CustomLidar::mount_position_compensation(pcl::PointXYZ const& point, pcl::PointXYZ& calibrated_pt) {
    calibrated_pt.x = point.x * cos(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
                      point.y * (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) -
                                 sin(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
                      point.z * (cos(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) +
                                 sin(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
                      this->LP.mp.x;
    calibrated_pt.y = point.x * sin(this->LP.mp.yaw) * cos(this->LP.mp.pitch) +
                      point.y * (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
                                 cos(this->LP.mp.yaw) * cos(this->LP.mp.roll)) +
                      point.z * (sin(this->LP.mp.yaw) * sin(this->LP.mp.pitch) * cos(this->LP.mp.roll) -
                                 cos(this->LP.mp.yaw) * sin(this->LP.mp.roll)) +
                      this->LP.mp.y;
    calibrated_pt.z = point.x * (-sin(this->LP.mp.pitch)) + point.y * cos(this->LP.mp.pitch) * sin(this->LP.mp.roll) +
                      point.z * cos(this->LP.mp.pitch) * cos(this->LP.mp.roll) + this->LP.mp.z;
    return ErrorCode::Success;
}
ErrorCode CustomLidar::arange(double const& minVal, double const& maxVal, double const& resolution,
                              std::vector<double>& vec) {
    // validate input arguments.
    if (!vec.empty()) {
        return ErrorCode::BadArgument;
    }
    // We get a vector varying from min val to max val with a step resolution.
    double minimum = std::min(minVal, maxVal);
    double maximum = std::max(minVal, maxVal);
    for (double i = minimum; i < maximum; i += resolution) {
        vec.push_back(i);
    }
    return ErrorCode::Success;
}
