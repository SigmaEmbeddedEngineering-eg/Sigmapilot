/*
 * Desc: Laser Scan fusion.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 30 march 2021
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

#include "scan_fusion.hpp"
scan_fusion::scan_fusion() {
    //
}
scan_fusion::~scan_fusion() {}

ErrorCode scan_fusion::scans_accumulator(pcl::PointCloud<pcl::PointXYZ>::Ptr const& cloud, int const& offset,
                                         tf::StampedTransform const& transform, std::vector<scan>& fused_ls) {
    float origin_distance =
        std::sqrt(std::pow(transform.getOrigin().x(), 2) + std::pow(transform.getOrigin().y(), 2)) + 0.1;

    for (int i = 0; i < cloud->points.size(); i++) {
        int sign = ((0 < cloud->points[i].y) - ((cloud->points[i].y < 0)));
        if (cloud->points[i].y * sign > 0) {
            scan temp_scan;
            temp_scan.x = cloud->points[i].x;
            temp_scan.y = cloud->points[i].y;
            temp_scan.yaw = std::atan2(cloud->points[i].y, cloud->points[i].x);
            temp_scan.depth = std::sqrt(std::pow(cloud->points[i].y, 2) + std::pow(cloud->points[i].x, 2));
            temp_scan.pointIdx = i + offset;
            if (temp_scan.pointIdx == 0)
                fused_ls.push_back(temp_scan);
            else if ((temp_scan.yaw > fused_ls[fused_ls.size() - 1].yaw) && temp_scan.depth > origin_distance)
                fused_ls.push_back(temp_scan);
        }
    }
    return ErrorCode::Success;
}
ErrorCode scan_fusion::fuse_scans(std::vector<scan> const& fused_ls, fusion_config const& config,
                                  sensor_msgs::LaserScan& fused_scans) {
    fused_scans.header.stamp = ros::Time::now();
    fused_scans.header.frame_id = config.fusion_frame;
    fused_scans.angle_min = fused_ls[0].yaw;
    fused_scans.angle_max = fused_ls[fused_ls.size() - 1].yaw;
    fused_scans.angle_increment = (fused_scans.angle_max - fused_scans.angle_min) / config.number_fused_points;
    fused_scans.range_min = config.range_min;
    fused_scans.range_max = config.range_max;

    fused_scans.ranges.resize(config.number_fused_points);
    std::fill(fused_scans.ranges.begin(), fused_scans.ranges.end(), fused_scans.range_max);
    fused_scans.intensities.resize(config.number_fused_points);
    std::fill(fused_scans.intensities.begin(), fused_scans.intensities.end(), 0);

    int non_fused_scan_idx = 0;
    float theta_lower_range = fused_ls[non_fused_scan_idx].yaw;
    float theta_upper_range = fused_ls[non_fused_scan_idx + 1].yaw;
    float theta = fused_scans.angle_min;
    for (int i = 0; i < config.number_fused_points; i++) {
        bool exit_condition = false;
        while (theta_upper_range < fused_scans.angle_max && !exit_condition) {
            if (theta <= theta_upper_range && theta >= theta_lower_range) {
                if (((fused_ls[non_fused_scan_idx + 1].pointIdx - fused_ls[non_fused_scan_idx].pointIdx) > 1))
                    fused_scans.ranges[i] = fused_scans.range_max;
                else if ((abs(fused_ls[non_fused_scan_idx + 1].depth - fused_ls[non_fused_scan_idx].depth) >
                          config.dis_threshold))
                    fused_scans.ranges[i] = fused_scans.range_max;
                else {
                    fused_scans.ranges[i] =
                        (theta - theta_lower_range) *
                            (fused_ls[non_fused_scan_idx + 1].depth - fused_ls[non_fused_scan_idx].depth) /
                            (theta_upper_range - theta_lower_range) +
                        fused_ls[non_fused_scan_idx].depth;
                }

                theta = theta + fused_scans.angle_increment;
                exit_condition = true;
            } else {
                non_fused_scan_idx += 1;
                theta_lower_range = fused_ls[non_fused_scan_idx].yaw;
                theta_upper_range =
                    fused_ls[std::min(non_fused_scan_idx + 1, static_cast<int>(fused_ls.size() - 1))].yaw;
            }
        }
    }
    return ErrorCode::Success;
}
