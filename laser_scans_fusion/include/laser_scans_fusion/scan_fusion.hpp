/*
 * Desc: Laser Scan fusion.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@robotion.ai
 * Date: 30 march 2021
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

#ifndef Scan_FUSION_HH
#define Scan_FUSION_HH
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "dataContainers.hpp"
class scan_fusion
{
public:
    scan_fusion();
    ~scan_fusion();
    void scans_accumulator(std::vector<scan> &fused_ls,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int cloud_offset,
                           tf::StampedTransform transform);
    sensor_msgs::LaserScan fuse_scans(std::vector<scan> fused_ls, fusion_config config);
};
#endif