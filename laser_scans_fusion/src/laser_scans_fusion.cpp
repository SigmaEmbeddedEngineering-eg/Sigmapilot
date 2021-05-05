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

#include "laser_scans_fusion.hpp"

Multi_Scan_Fusion::Multi_Scan_Fusion() {
    this->node_.getParam(ros::this_node::getName() + "/fusion_frame", this->config.fusion_frame);
    this->node_.getParam(ros::this_node::getName() + "/number_fused_points", this->config.number_fused_points);
    this->node_.getParam(ros::this_node::getName() + "/range_min", this->config.range_min);
    this->node_.getParam(ros::this_node::getName() + "/dis_threshold", this->config.dis_threshold);
    this->node_.getParam(ros::this_node::getName() + "/tf_listener_rate", this->config.tf_listener_rate);

    std::string input_topic1, input_topic2, fused_topic;
    this->node_.getParam(ros::this_node::getName() + "/input_topic1", input_topic1);
    this->node_.getParam(ros::this_node::getName() + "/input_topic2", input_topic2);
    this->node_.getParam(ros::this_node::getName() + "/fused_topic", fused_topic);

    tfListener_.setExtrapolationLimit(ros::Duration(this->config.tf_listener_rate));

    scanSub1.subscribe(node_, input_topic1, 1);
    scanSub2.subscribe(node_, input_topic2, 1);

    sync_.reset(new Sync(MySyncPolicy(10), scanSub1, scanSub2));
    sync_->registerCallback(boost::bind(&Multi_Scan_Fusion::scanCallback, this, _1, _2));

    laserscan_publisher_ = node_.advertise<sensor_msgs::LaserScan>(fused_topic, 1, false);
}
Multi_Scan_Fusion::~Multi_Scan_Fusion() {}

void Multi_Scan_Fusion::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan1,
                                     const sensor_msgs::LaserScan::ConstPtr &scan2) {
    tf::StampedTransform transform, transform2;
    tfListener_.lookupTransform(this->config.fusion_frame, scan1->header.frame_id, ros::Time(0), transform);
    tfListener_.lookupTransform(this->config.fusion_frame, scan2->header.frame_id, ros::Time(0), transform2);

    sensor_msgs::PointCloud2 cloud, cloud2;
    projector_.transformLaserScanToPointCloud(this->config.fusion_frame, *scan1, cloud, tfListener_);
    projector_.transformLaserScanToPointCloud(this->config.fusion_frame, *scan2, cloud2, tfListener_);

    pcl::PCLPointCloud2 pcl_pc, pcl_pc2;
    pcl_conversions::toPCL(cloud, pcl_pc);
    pcl_conversions::toPCL(cloud2, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc, *temp_cloud);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud2);

    std::vector<scan> fused_ls;
    (void)sf.scans_accumulator(temp_cloud, 0, transform, fused_ls);
    (void)sf.scans_accumulator(temp_cloud2, fused_ls.size(), transform2, fused_ls);

    config.range_max =
        std::max(scan1->range_max, scan2->range_max) +
        std::max(std::sqrt(std::pow(transform.getOrigin().x(), 2) + std::pow(transform.getOrigin().y(), 2)),
                 std::sqrt(std::pow(transform2.getOrigin().x(), 2) + std::pow(transform2.getOrigin().y(), 2))) +
        0.1;
    sensor_msgs::LaserScan fused_scans;
    (void)sf.fuse_scans(fused_ls, config, fused_scans);
    laserscan_publisher_.publish(fused_scans);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "laser_scans_fusion");

    std::cout << "initialization" << std::endl;

    Multi_Scan_Fusion filter;

    ros::spin();

    return 0;
}