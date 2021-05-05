/*
 * Desc: SGM LiDAR clustering.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 12 September 2020
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <ctime>
#include <numeric>
#include <opencv2/highgui.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/tracking.hpp>
#include <string>
#include "dataContainers.hpp"
#include "ground_segmentation.hpp"
#include "sgm_segmentation.hpp"
class SGMClustering {
 public:
  SGMClustering();
  ~SGMClustering();
  ros::Publisher pub;
  ros::Publisher laser_scan_pub;
  ros::Publisher marker_pub;
  
  void cloud_cb(const PointCloud::ConstPtr& input);

  ros::NodeHandle nh;
  ros::Subscriber sub;
  configuration config;
  std::unique_ptr<GroundSegmentation> gs_ptr;
  std::unique_ptr<SGMSegmentation> sgm_ss_ptr;

};