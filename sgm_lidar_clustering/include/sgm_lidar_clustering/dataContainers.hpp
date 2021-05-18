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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#ifndef DATA_CONTAINER_HH
#define DATA_CONTAINER_HH
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud_C;

class SGM
{
public:
  cv::Mat x, y, z, r, g, b, d;
  int height, width;
  SGM(int height, int width)
  {
    this->x = cv::Mat(height, width, CV_32F);
    this->y = cv::Mat(height, width, CV_32F);
    this->z = cv::Mat(height, width, CV_32F);
    this->r = cv::Mat(height, width, CV_32F);
    this->g = cv::Mat(height, width, CV_32F);
    this->b = cv::Mat(height, width, CV_32F);
    this->d = cv::Mat(height, width, CV_32F);
    this->height = height;
    this->width = width;
  }
  void Set(const PointCloud::ConstPtr input)
  {
    for (int rowIdx = 0; rowIdx < this->height; rowIdx++)
    {
      for (int colIdx = 0; colIdx < this->width; colIdx++)
      {
        int pointIdx = colIdx + this->width * rowIdx;
        this->x.at<float>(rowIdx, colIdx) = input->points[pointIdx].x;
        this->y.at<float>(rowIdx, colIdx) = input->points[pointIdx].y;
        this->z.at<float>(rowIdx, colIdx) = input->points[pointIdx].z;
        this->r.at<float>(rowIdx, colIdx) = input->points[pointIdx].r;
        this->g.at<float>(rowIdx, colIdx) = input->points[pointIdx].g;
        this->b.at<float>(rowIdx, colIdx) = input->points[pointIdx].b;
        this->d.at<float>(rowIdx, colIdx) =
            sqrt(input->points[pointIdx].x * input->points[pointIdx].x +
                 input->points[pointIdx].y * input->points[pointIdx].y +
                 input->points[pointIdx].z * input->points[pointIdx].z);
      }
    }
  }
};

class configuration
{
public:
  std::string LiDAR_topicname;
  std::string pub_topicname;
  std::string pub_Objtopicname;
  bool obj_level_filter_flag;
  double ground_segmentation_threshold;
  double distance_threshold;
  double distance_threshold_type;
  double obj_level_x_min;
  double obj_level_y_min;
  double obj_level_z_min;
  double obj_level_x_max;
  double obj_level_y_max;
  double obj_level_z_max;
  double obj_level_vol_threshold;
  double min_horizontal_angle;
  double max_horizontal_angle;
  double lidar_max_range;
  bool use_morphological_filter;
};

class object
{
public:
  double pos_x;
  double pos_y;
  double pos_z;
  double size_x;
  double size_y;
  double size_z;
};

class clustered_objects
{
public:
  cv::Mat labels;
  std::vector<object> objects;
};
#endif