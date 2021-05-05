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

#include "point_cloud_assembler.hpp"

pointCloudAssembler::pointCloudAssembler() {
    std::cout << "initialize" << std::endl;

    std::string pub_topicname = "assemblerMap";
    this->nh.getParam(ros::this_node::getName() + "/pub_topicname", pub_topicname);
    this->nh.getParam(ros::this_node::getName() + "/voxel_size", this->config.voxel_size);
    this->nh.getParam(ros::this_node::getName() + "/tf_duration", this->config.tf_duration);
    this->nh.getParam(ros::this_node::getName() + "/odom_frame", this->config.odom_frame);

    tfListener_.setExtrapolationLimit(ros::Duration(this->config.tf_duration));

    this->pub = this->nh.advertise<PointCloud_C>(pub_topicname, 1);

    this->cloud_accumilated.width = 0;
    this->cloud_accumilated.height = 0;
    this->cloud_accumilated.points.clear();
    XmlRpc::XmlRpcValue v;
    nh.param("/LiDAR_topicname", v, v);
    for (int i = 0; i < v.size(); i++) {
        sub_vec.push_back(this->nh.subscribe<PointCloud>(v[i], 1, &pointCloudAssembler::cloud_cb, this));
        std::cout << "topic_names: " << v[i] << std::endl;
    }
    ros::Duration(this->config.tf_duration * 2).sleep();
}
pointCloudAssembler::~pointCloudAssembler() {}

void pointCloudAssembler::cloud_cb(const PointCloud::ConstPtr &input) {
    // down sample the point cloud
    PointCloud::Ptr downsampledPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(this->config.voxel_size, this->config.voxel_size, this->config.voxel_size);
    sor.filter(*downsampledPC);

    pcl::PointCloud<pcl::PointXYZRGB> cloud_in = *downsampledPC;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_trans;
    tf::StampedTransform transform, transform2;

    tfListener_.lookupTransform(this->config.odom_frame, input->header.frame_id, ros::Time(0), transform);

    // translate and rotate the downsampled point cloud.
    pcl_ros::transformPointCloud(cloud_in, cloud_trans, transform);
    // assemble point cloud.
    this->cloud_accumilated.width = this->cloud_accumilated.width + cloud_trans.width;
    this->cloud_accumilated.height = 1;
    this->cloud_accumilated.points.insert(this->cloud_accumilated.points.end(), cloud_trans.points.begin(),
                                          cloud_trans.points.end());

    std::cout << "PointCloud after filtering: " << downsampledPC->width << "  " << downsampledPC->height
              << "  before filtering:" << input->width * input->height
              << "  accumilative filtering:" << this->cloud_accumilated.width * this->cloud_accumilated.height
              << std::endl;

    // down sample the assembled point cloud.
    PointCloud::Ptr accumilatedPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    PointCloud::Ptr downsampledAccumilatedPC(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor_accumilated;
    *accumilatedPC = this->cloud_accumilated;
    sor_accumilated.setInputCloud(accumilatedPC);
    sor_accumilated.setLeafSize(this->config.voxel_size, this->config.voxel_size, this->config.voxel_size);
    sor_accumilated.filter(*downsampledAccumilatedPC);
    this->cloud_accumilated = *downsampledAccumilatedPC;
    this->cloud_accumilated.header = input->header;
    this->cloud_accumilated.header.frame_id = this->config.odom_frame;
    // publish point cloud.
    this->pub.publish(this->cloud_accumilated);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "point_cloud_assembler");

    // Create a pointcloud assembler object.
    pointCloudAssembler assembler;

    // Spin
    ros::spin();
}
