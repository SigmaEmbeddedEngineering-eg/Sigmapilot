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

#ifndef DATA_CONTAINER_HH
#define DATA_CONTAINER_HH
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGBL> PointCloud_C;
typedef pcl::PointCloud<pcl::PointXYZL> MapPointCloud;

class configuration {
   public:
    double voxel_size;
    double min_distance_range;
    int freq_ratio;
    std::string odom_frame;
    double slope_threshold;
    double euclidean_fitness_epsilon;
    double transformation_epsilon;
    double maximum_iterations;
    double max_correspondence_distance;
    double rejection_threshold;
    double map_score_threshold;
    double crop_box_threshold;
    bool use_external_imu;
};
class euler_rot {
   public:
    double roll, pitch, yaw;
    euler_rot() : roll{0}, pitch{0}, yaw{0} {}
};

class odom {
   public:
    double x, y, z, roll, pitch, yaw;
    odom() {
        this->x = 0;
        this->y = 0;
        this->z = 0;
    }
    void update_xyz(double dx, double dy, double dz) {
        this->x += dx;
        this->y += dy;
        this->z += dz;
    }
    void print_state() { std::cout << "odom: " << x << ", " << y << ", " << z << std::endl << std::endl; }
};
class SGM {
   public:
    cv::Mat x, y, z, r, g, b, d;
    int height, width;
    SGM(int height, int width) {
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
    void Set(const PointCloud::ConstPtr input) {
        for (int rowIdx = 0; rowIdx < this->height; rowIdx++) {
            for (int colIdx = 0; colIdx < this->width; colIdx++) {
                int pointIdx = colIdx + this->width * rowIdx;
                this->x.at<float>(rowIdx, colIdx) = input->points[pointIdx].x;
                this->y.at<float>(rowIdx, colIdx) = input->points[pointIdx].y;
                this->z.at<float>(rowIdx, colIdx) = input->points[pointIdx].z;
                this->r.at<float>(rowIdx, colIdx) = input->points[pointIdx].r;
                this->g.at<float>(rowIdx, colIdx) = input->points[pointIdx].g;
                this->b.at<float>(rowIdx, colIdx) = input->points[pointIdx].b;
                this->d.at<float>(rowIdx, colIdx) = sqrt(input->points[pointIdx].x * input->points[pointIdx].x +
                                                         input->points[pointIdx].y * input->points[pointIdx].y +
                                                         input->points[pointIdx].z * input->points[pointIdx].z);
            }
        }
    }
};

enum ErrorCode { Success, Failed, Unknown, BadArgument };

#endif