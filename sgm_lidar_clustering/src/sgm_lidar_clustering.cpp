/*
 * Desc: SGM LiDAR clustering.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@robotion.ai
 * Date: 12 September 2020
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

#include "sgm_lidar_clustering.hpp"

SGMClustering::SGMClustering()
{
  std::cout << "initialize" << std::endl;
  this->nh.getParam(ros::this_node::getName() + "/LiDAR_topicname",
                    this->config.LiDAR_topicname);
  this->nh.getParam(ros::this_node::getName() + "/pub_pc_topicname",
                    this->config.pub_topicname);
  this->nh.getParam(ros::this_node::getName() + "/pub_ma_topicname",
                    this->config.pub_Objtopicname);

  this->nh.getParam(
      ros::this_node::getName() + "/ground_segmentation_threshold",
      this->config.ground_segmentation_threshold);

  this->nh.getParam(ros::this_node::getName() + "/distance_threshold",
                    this->config.distance_threshold);
  this->nh.getParam(ros::this_node::getName() + "/distance_threshold_type",
                    this->config.distance_threshold_type);

  this->nh.getParam(ros::this_node::getName() + "/obj_level_filter_flag",
                    this->config.obj_level_filter_flag);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_x_min",
                    this->config.obj_level_x_min);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_y_min",
                    this->config.obj_level_y_min);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_z_min",
                    this->config.obj_level_z_min);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_x_max",
                    this->config.obj_level_x_max);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_y_max",
                    this->config.obj_level_y_max);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_z_max",
                    this->config.obj_level_z_max);
  this->nh.getParam(ros::this_node::getName() + "/obj_level_vol_threshold",
                    this->config.obj_level_vol_threshold);
  this->nh.getParam(ros::this_node::getName() + "/min_horizontal_angle",
                    this->config.min_horizontal_angle);
  this->nh.getParam(ros::this_node::getName() + "/max_horizontal_angle",
                    this->config.max_horizontal_angle);
  this->nh.getParam(ros::this_node::getName() + "/lidar_max_range",
                    this->config.lidar_max_range);

  std::cout << "obj_level_filter_flag: " << this->config.obj_level_filter_flag
            << std::endl;
  std::cout << "gnd_seg_threshold: "
            << this->config.ground_segmentation_threshold << std::endl;
  std::cout << "distance_threshold: " << this->config.distance_threshold
            << std::endl;
  std::cout << "point cloud sub topic: " << this->config.LiDAR_topicname
            << std::endl;
  std::cout << "point cloud pub topic: " << this->config.pub_topicname
            << std::endl;
  std::cout << "object pub topic: " << this->config.pub_Objtopicname
            << std::endl;

  this->sub = this->nh.subscribe<PointCloud>(this->config.LiDAR_topicname, 1,
                                             &SGMClustering::cloud_cb, this);
  this->pub = this->nh.advertise<PointCloud_C>(this->config.pub_topicname, 1);
  this->laser_scan_pub = this->nh.advertise<sensor_msgs::LaserScan>(this->config.pub_topicname + "_laser_scaner", 1);
  this->marker_pub = this->nh.advertise<visualization_msgs::MarkerArray>(
      this->config.pub_Objtopicname, 1);
  this->gs_ptr = std::make_unique<GroundSegmentation>(this->config);
  this->sgm_ss_ptr = std::make_unique<SGMSegmentation>(this->config);
}
SGMClustering::~SGMClustering() {}
void SGMClustering::cloud_cb(const PointCloud::ConstPtr &input)
{
  //
  clock_t begin = clock();
  // Map the point cloud from Pointcloud container to Spherical grid map
  // representation, where it is a matrix form with
  SGM sgm(input->height, input->width);

  // Set Spherical grid map containers with the point cloud.
  sgm.Set(input);
  cv::Mat ground_label = this->gs_ptr->ground_segmentation(sgm);
  std::vector<float> freespace_d = this->gs_ptr->freespace(ground_label, sgm);
  clustered_objects labels = this->sgm_ss_ptr->semantic_segmentation(sgm, ground_label);

  visualization_msgs::MarkerArray ma;
  for (int i = 0; i < labels.objects.size(); i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = input->header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "object";
    marker.id = i;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.text = "Detected Obj" + std::to_string(i);

    marker.pose.position.x = labels.objects[i].pos_x;
    marker.pose.position.y = labels.objects[i].pos_y;
    marker.pose.position.z = labels.objects[i].pos_z;
    marker.scale.x = labels.objects[i].size_x;
    marker.scale.y = labels.objects[i].size_y;
    marker.scale.z = labels.objects[i].size_z;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration(0.1);
    ma.markers.push_back(marker);
  }
  this->marker_pub.publish(ma);

  PointCloud_C pcl_msg;
  pcl_msg.header = input->header;
  for (int rowIdx = 0; rowIdx < sgm.height; rowIdx++)
  {
    for (int colIdx = 0; colIdx < sgm.width; colIdx++)
    {
      pcl::PointXYZRGBL pt;
      pt = pcl::PointXYZRGBL();
      pt.x = sgm.x.at<float>(rowIdx, colIdx);
      pt.y = sgm.y.at<float>(rowIdx, colIdx);
      pt.z = sgm.z.at<float>(rowIdx, colIdx);
      pt.r = sgm.r.at<float>(rowIdx, colIdx);
      pt.g = sgm.g.at<float>(rowIdx, colIdx);
      pt.b = sgm.b.at<float>(rowIdx, colIdx);
      pt.label = labels.labels.at<float>(rowIdx, colIdx);
      pcl_msg.points.push_back(pt);
    }
  }

  // Publish the data.
  this->pub.publish(pcl_msg);

  sensor_msgs::LaserScan scans;
  scans.header.stamp = ros::Time::now();
  scans.header.frame_id = input->header.frame_id;
  scans.angle_min = this->config.min_horizontal_angle;
  scans.angle_max = this->config.max_horizontal_angle;
  scans.angle_increment = (this->config.max_horizontal_angle -
                           this->config.min_horizontal_angle) /
                          sgm.width;
  //scans.time_increment = (1 / 10) / (input->width);
  scans.range_min = 0.0;
  scans.range_max = this->config.lidar_max_range;
  scans.ranges = freespace_d;
  std::vector<float> intensities(sgm.width, 0);
  scans.intensities = intensities;
  this->laser_scan_pub.publish(scans);

  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Elapsed_time: " << elapsed_secs
            << " sec, Freq:" << 1 / elapsed_secs << " Hz" << std::endl;
  std::cout << "******************************************************" << std::endl;
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, "sgm_lidar_clustering");

  // Create a LiDAR clustering object.
  SGMClustering cluster;

  // Spin
  ros::spin();
}
