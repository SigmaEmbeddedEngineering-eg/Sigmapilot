/*
 * Desc: feature_based_mapping_icp.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 10 April 2021
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

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_pointvector.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ctime>
#include <mutex>  // std::mutex
#include <pcl_ros/impl/transforms.hpp>
#include <string>

#include "dataContainers.hpp"
#include "sensor_msgs/Imu.h"

class fbm_icp {
   public:
    fbm_icp();
    ~fbm_icp();
    ros::Publisher pub;
    ros::Publisher stampedPosePub;
    ros::Publisher LaserOdometryPub;
    void cloud_cb(const PointCloud::ConstPtr& input);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
    ErrorCode feature_extraction(SGM const& sgm, MapPointCloud& keypoints);
    void icp_mapping(std::pair<MapPointCloud, euler_rot> const& key_points_pair);

    ros::NodeHandle nh;
    ros::Subscriber imu_sub;
    std::vector<ros::Subscriber> sub_vec;
    std::vector<std::pair<MapPointCloud, euler_rot>> keypoints_vec;
    std::vector<PointCloud> dense_pc_vec;
    MapPointCloud map;
    PointCloud dense_map;
    configuration config;

    odom odom_;
    euler_rot rot_;
    std::mutex mtx_map_odom, mtx_keypoints, mtx_imu;
};