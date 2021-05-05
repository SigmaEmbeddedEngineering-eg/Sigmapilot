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

#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "dataContainers.hpp"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "scan_fusion.hpp"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"

using namespace message_filters;

class Multi_Scan_Fusion {
   public:
    Multi_Scan_Fusion();
    ~Multi_Scan_Fusion();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan1, const sensor_msgs::LaserScan::ConstPtr &scan2);

   private:
    ros::NodeHandle node_;
    laser_geometry::LaserProjection projector_;
    tf::TransformListener tfListener_;

    // ros::Publisher point_cloud_publisher_, point_cloud_publisher2_;
    ros::Publisher laserscan_publisher_;
    ros::Subscriber scan_sub_;

    message_filters::Subscriber<sensor_msgs::LaserScan> scanSub1;
    message_filters::Subscriber<sensor_msgs::LaserScan> scanSub2;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan>
        MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> Sync;
    boost::shared_ptr<Sync> sync_;
    scan_fusion sf;
    fusion_config config;
};