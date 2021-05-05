/*
 * Desc: Pointcloud assembler.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 14 September 2020
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

#include <geometry_msgs/TransformStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <string>

#include "dataContainers.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud_C;
class pointCloudAssembler {
   public:
    pointCloudAssembler();
    ~pointCloudAssembler();
    ros::Publisher pub;
    void cloud_cb(const PointCloud::ConstPtr &input);

    ros::NodeHandle nh;
    std::vector<ros::Subscriber> sub_vec;
    tf::TransformListener tfListener_;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_accumilated;
    configuration config;
};