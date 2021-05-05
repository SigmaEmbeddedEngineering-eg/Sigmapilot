/*
 * Desc: LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@sigma.se
 * Date: 5 September 2020
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

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <algorithm>
#include <sstream>
#include <string>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class mountingPositions {
   public:
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
    void Set(std::string sensor_pose) {
        std::stringstream ss(sensor_pose);
        ss >> this->x;
        ss >> this->y;
        ss >> this->z;
        ss >> this->roll;
        ss >> this->pitch;
        ss >> this->yaw;
    };
};

class LidarParameters {
   public:
    float min_vertical_angle;
    float max_vertical_angle;
    float vertical_step_angle;

    float min_horizontal_angle;
    float max_horizontal_angle;
    float horizontal_step_angle;

    float horizontal_points_count;
    float vertical_points_count;

    bool parameters_initialized;

    float cam_fov;
    bool enable_fusion;

    mountingPositions mp;

    std::vector<double> azimuthVec, elevationVec;
};

enum ErrorCode { Success, Failed, Unknown, BadArgument };

#endif
