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
/*
 * steps:
 * ------
 * 1) ponint cloud encoding in sgm.
 * 2) feature extraction of the slopes diffrentiation.
 * 3) encode the keypoints in XYZRGB
 * 4) downsample the point cloud
 * 5) translate and rotate the current point cloud with the previous location.
 * 6) calculate the translation and rotaiton with the old map
 * 7) update the localization of the vehicle.
 * 8) combine the two pointclouds.
 * 9) downsample the combined pointcloud again.
 *10) publish the constructed map.
 */

#include "fbm_icp.hpp"
#include <typeinfo>

fbm_icp::fbm_icp() {
    std::cout << "initialize" << std::endl;

    std::string pub_topicname = "fbm_icp_map";
    std::string pub_posetopicname = "map_pose";
    std::string imu_topicname = "imu";
    this->nh.getParam(ros::this_node::getName() + "/pub_topicname", pub_topicname);
    this->nh.getParam(ros::this_node::getName() + "/pub_posetopicname", pub_posetopicname);
    this->nh.getParam(ros::this_node::getName() + "/imu_topicname", imu_topicname);
    this->nh.getParam(ros::this_node::getName() + "/voxel_size", this->config.voxel_size);
    this->nh.getParam(ros::this_node::getName() + "/odom_frame", this->config.odom_frame);
    this->nh.getParam(ros::this_node::getName() + "/freq_ratio", this->config.freq_ratio);
    this->nh.getParam(ros::this_node::getName() + "/slope_threshold", this->config.slope_threshold);
    this->nh.getParam(ros::this_node::getName() + "/min_distance_range", this->config.min_distance_range);
    this->nh.getParam(ros::this_node::getName() + "/euclidean_fitness_epsilon", this->config.euclidean_fitness_epsilon);
    this->nh.getParam(ros::this_node::getName() + "/transformation_epsilon", this->config.transformation_epsilon);
    this->nh.getParam(ros::this_node::getName() + "/maximum_iterations", this->config.maximum_iterations);
    this->nh.getParam(ros::this_node::getName() + "/max_correspondence_distance",
                      this->config.max_correspondence_distance);
    this->nh.getParam(ros::this_node::getName() + "/rejection_threshold", this->config.rejection_threshold);
    this->nh.getParam(ros::this_node::getName() + "/map_score_threshold", this->config.map_score_threshold);
    this->nh.getParam(ros::this_node::getName() + "/crop_box_threshold", this->config.crop_box_threshold);
    this->nh.getParam(ros::this_node::getName() + "/use_external_imu", this->config.use_external_imu);

    //[x] TODO:
    // read initial position from here.
    std::vector<double> fbm_initial_pose;
    nh.param("/fbm_initial_pose", fbm_initial_pose,fbm_initial_pose);
    
    odom_.x = fbm_initial_pose[0];
    odom_.y = fbm_initial_pose[1];
    odom_.z = fbm_initial_pose[2];
    rot_.roll = fbm_initial_pose[3];
    rot_.pitch = fbm_initial_pose[4];
    rot_.yaw = fbm_initial_pose[5];
    
    this->pub = this->nh.advertise<PointCloud_C>(pub_topicname, 1);
    this->stampedPosePub = this->nh.advertise<geometry_msgs::PoseStamped>(pub_posetopicname, 1);
    this->LaserOdometryPub = this->nh.advertise<nav_msgs::Odometry>(pub_posetopicname + "_odom", 1);

    this->map.width = 0;
    this->map.height = 0;
    this->map.points.clear();
    XmlRpc::XmlRpcValue v;
    nh.param("/LiDAR_topicname", v, v);
    for (int i = 0; i < v.size(); i++) {
        sub_vec.push_back(this->nh.subscribe<PointCloud>(v[i], 1, &fbm_icp::cloud_cb, this));
        std::cout << "topic_names: " << v[i] << std::endl;
    }
    //[x] TODO: take imu topic from lunch file.
    imu_sub = this->nh.subscribe(imu_topicname, 1, &fbm_icp::imu_cb, this);
}
fbm_icp::~fbm_icp() {}

ErrorCode fbm_icp::feature_extraction(SGM const &sgm, MapPointCloud &keypoints) {
    // Feature Extractions:
    // revise step1: sqrt((x-x)²+(y-y)²)/abs(z-z)
    // slope left to right
    cv::Mat deltaX = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::pow(sgm.x(cv::Range(1, sgm.x.rows), cv::Range::all()) - sgm.x(cv::Range(0, sgm.x.rows - 1), cv::Range::all()),
            2, deltaX(cv::Range(1, sgm.x.rows), cv::Range::all()));

    cv::Mat deltaY = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::pow(sgm.y(cv::Range(1, sgm.x.rows), cv::Range::all()) - sgm.y(cv::Range(0, sgm.x.rows - 1), cv::Range::all()),
            2, deltaY(cv::Range(1, sgm.x.rows), cv::Range::all()));
    cv::Mat deltaXY = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);

    cv::Mat deltaZ = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::pow(sgm.z(cv::Range(1, sgm.x.rows), cv::Range::all()) - sgm.z(cv::Range(0, sgm.x.rows - 1), cv::Range::all()),
            2, deltaZ(cv::Range(1, sgm.x.rows), cv::Range::all()));
    cv::sqrt(deltaX + deltaY + deltaZ, deltaXY);
    // cv::sqrt(deltaX + deltaY, deltaXY);
    cv::sqrt(deltaZ, deltaZ);

    cv::Mat slopeUp = deltaZ / deltaXY;
    cv::Mat slopes_diff = cv::Mat::zeros(sgm.x.rows, sgm.x.cols, CV_32F);
    cv::absdiff(slopeUp(cv::Range(1, sgm.x.rows), cv::Range::all()),
                slopeUp(cv::Range(0, sgm.x.rows - 1), cv::Range::all()),
                slopes_diff(cv::Range(1, sgm.x.rows), cv::Range::all()));

    cv::threshold(slopes_diff, slopes_diff, this->config.slope_threshold, 1, 0);
    slopes_diff.convertTo(slopes_diff, CV_8UC1);
    std::vector<cv::Point> locations;  // output, locations of non-zero pixels
    cv::findNonZero(slopes_diff, locations);

    MapPointCloud differential_pointcloud;
    bool map_is_empty = (this->map.points.size() == 0);
    // assemble point cloud.
    for (int i = 0; i < locations.size(); i++) {
        pcl::PointXYZL pt;
        pt.x = sgm.x.at<float>(locations[i].y, locations[i].x);
        pt.y = sgm.y.at<float>(locations[i].y, locations[i].x);
        pt.z = sgm.z.at<float>(locations[i].y, locations[i].x);
        // [x]TODO:
        // if map is empty set the label to the threshold + 1.
        if (map_is_empty)
            pt.label = this->config.map_score_threshold + 1;
        else
            pt.label = 1;
        double dis = sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2));
        if (dis > this->config.min_distance_range) differential_pointcloud.points.push_back(pt);
    }
    // down sample the point cloud
    MapPointCloud::Ptr downsampledPC(new MapPointCloud);
    MapPointCloud::Ptr cloud(new MapPointCloud);
    *cloud = differential_pointcloud;
    pcl::VoxelGrid<pcl::PointXYZL> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(this->config.voxel_size, this->config.voxel_size, this->config.voxel_size);
    sor.filter(*downsampledPC);
    keypoints = *downsampledPC;
    return ErrorCode::Success;
}

void fbm_icp::icp_mapping(std::pair<MapPointCloud, euler_rot> const &key_points_pair) {
    // if there is no map, then start the map with the current cloud,
    // otherwise calculate the translation.
    // copy map, and odom to local containers
    mtx_map_odom.lock();
    MapPointCloud current_map = this->map;
    odom odometry = this->odom_;
    mtx_map_odom.unlock();

    euler_rot euler_rot_ = key_points_pair.second;

    if (current_map.points.size() == 0) {
        //[x]TODO: compensate the initial pose.
        MapPointCloud cloud_ds = key_points_pair.first;

        // compensate the accumlation of motion in the previous scans.
        tf::Transform transform_;
        transform_.setOrigin(tf::Vector3(odometry.x, odometry.y, odometry.z));
        tf::Quaternion q;
        q.setRPY(euler_rot_.roll, euler_rot_.pitch, euler_rot_.yaw);
        transform_.setRotation(q);
        MapPointCloud cloud_trans;
        tf::StampedTransform tf_(transform_, ros::Time::now(), "map", "robot");
        pcl_ros::transformPointCloud(cloud_ds, cloud_trans, tf_);
        current_map = cloud_trans;
    } else {
        MapPointCloud cloud_ds = key_points_pair.first;

        MapPointCloud::Ptr cloud_in(new MapPointCloud);
        MapPointCloud::Ptr cloud_out(new MapPointCloud);

        // compensate the accumlation of motion in the previous scans.
        tf::Transform transform_;
        transform_.setOrigin(tf::Vector3(odometry.x, odometry.y, odometry.z));
        tf::Quaternion q;
        q.setRPY(euler_rot_.roll, euler_rot_.pitch, euler_rot_.yaw);
        transform_.setRotation(q);
        MapPointCloud cloud_trans;
        tf::StampedTransform tf_(transform_, ros::Time::now(), "map", "robot");
        pcl_ros::transformPointCloud(cloud_ds, cloud_trans, tf_);

        // [x]TODO:set a filter on the point cloud.
        // - set the cropped pcl as the input for the icp.
        Eigen::Vector4f v_min = Eigen::Vector4f(odometry.x - this->config.crop_box_threshold, odometry.y - this->config.crop_box_threshold,
                                                odometry.z - this->config.crop_box_threshold, 1.0f);

        Eigen::Vector4f v_max = Eigen::Vector4f(odometry.x + this->config.crop_box_threshold, odometry.y + this->config.crop_box_threshold,
                                                odometry.z + this->config.crop_box_threshold, 1.0f);

        MapPointCloud::Ptr features_cloud_input_ptr(new MapPointCloud);
        MapPointCloud::Ptr cropped_features_cloud_ptr(new MapPointCloud);
        *features_cloud_input_ptr = cloud_trans;
        // Apply Cropbox
        pcl::CropBox<pcl::PointXYZL> cropBox_features(true);
        cropBox_features.setMin(v_min);
        cropBox_features.setMax(v_max);
        cropBox_features.setInputCloud(features_cloud_input_ptr);
        cropBox_features.filter(*cropped_features_cloud_ptr);

        cloud_in = cropped_features_cloud_ptr;  // cloud_ds;
        *cloud_out = current_map;

        pcl::IterativeClosestPoint<pcl::PointXYZL, pcl::PointXYZL> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        //////////////
        icp.setEuclideanFitnessEpsilon(this->config.euclidean_fitness_epsilon);
        icp.setTransformationEpsilon(this->config.transformation_epsilon);
        icp.setMaximumIterations(this->config.maximum_iterations);
        icp.setMaxCorrespondenceDistance(this->config.max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(this->config.rejection_threshold);
        //////////
        MapPointCloud Final, unmatchedPoints;
        icp.align(Final);
        std::cout << "------------------------------------------------------------------------" << std::endl;
        std::cout << Final.points.size() << " " << cloud_out->points.size() << " " << cloud_in->points.size()
                  << std::endl;
        std::cout << "------------------------------------------------------------------------" << std::endl;

        auto trafo = icp.getFinalTransformation();
        Eigen::Transform<float, 3, Eigen::Affine> tROTA(trafo);
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(tROTA, x, y, z, roll, pitch, yaw);
        // [ ]TODO: odometry smothing/Filtering.
        odometry.update_xyz(x, y, z);
        odometry.print_state();
        //[ ]TODO: add external imu flag.

        // [x]TODO: update scores.
        // 1) Search by using Octree loop on each point in the current keypoints.
        // 2) use voxelSearch.
        // 3) update scores add the current pointcloud to the current map.
        float resolution = this->config.voxel_size;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> octree(resolution);
        octree.setInputCloud(cloud_out);
        octree.addPointsFromInputCloud();
        for (int pt_idx = 0; pt_idx < Final.size(); pt_idx++) {
            pcl::PointXYZL searchPoint;

            searchPoint.x = Final[pt_idx].x;
            searchPoint.y = Final[pt_idx].y;
            searchPoint.z = Final[pt_idx].z;
            searchPoint.label = Final[pt_idx].label;

            // Neighbors within voxel search
            std::vector<int> pointIdxVec;
            if (octree.isVoxelOccupiedAtPoint(searchPoint)) {
                if (octree.voxelSearch(searchPoint, pointIdxVec)) {
                    for (std::size_t i = 0; i < pointIdxVec.size(); ++i) {
                        current_map.points[pointIdxVec[i]].label += 1;
                        current_map.points[pointIdxVec[i]].label =
                            std::min(static_cast<int>(current_map.points[pointIdxVec[i]].label), static_cast<int>(20));
                    }
                } else {
                    unmatchedPoints.points.push_back(searchPoint);
                }
            } else {
                unmatchedPoints.points.push_back(searchPoint);
            }
        }

        current_map.insert(current_map.points.end(), unmatchedPoints.points.begin(), unmatchedPoints.points.end());

        // down sample the point cloud
        MapPointCloud::Ptr FinaldownsampledPC(new MapPointCloud);
        MapPointCloud::Ptr Finalcloud(new MapPointCloud);
        *Finalcloud = current_map;
        pcl::VoxelGrid<pcl::PointXYZL> sorFinal;
        sorFinal.setInputCloud(Finalcloud);
        sorFinal.setLeafSize(this->config.voxel_size, this->config.voxel_size, this->config.voxel_size);
        sorFinal.filter(*FinaldownsampledPC);
        current_map = *FinaldownsampledPC;
    }
    mtx_map_odom.lock();
    this->map = current_map;
    this->odom_ = odometry;
    mtx_map_odom.unlock();
}

void fbm_icp::imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y,
    //         msg->orientation.z, msg->orientation.w);
    tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    tf::Matrix3x3 m(q);
    mtx_imu.lock();
    m.getRPY(rot_.roll, rot_.pitch, rot_.yaw);
    mtx_imu.unlock();
    ROS_INFO("Imu Orientation R: [%f], P: [%f], Y: [%f]", rot_.roll, rot_.pitch, rot_.yaw);
}

void fbm_icp::cloud_cb(const PointCloud::ConstPtr &input) {
    // Map the point cloud from Pointcloud container to Spherical grid map
    // representation, where it is a matrix form with
    SGM sgm(input->height, input->width);

    // Set Spherical grid map containers with the point cloud.
    sgm.Set(input);

    // extract features
    MapPointCloud keypoints;
    (void)feature_extraction(sgm, keypoints);

    //[ ]TODO: add external imu flag.
    mtx_imu.lock();
    euler_rot euler_rot_ = rot_;
    mtx_imu.unlock();

    mtx_keypoints.lock();
    this->keypoints_vec.push_back(std::make_pair(keypoints, euler_rot_));
    mtx_keypoints.unlock();

    mtx_map_odom.lock();
    // fbm_icp_map.icp_mapping(key_points);
    MapPointCloud current_map = this->map;
    odom odometry = this->odom_;
    mtx_map_odom.unlock();

    current_map.header = input->header;
    current_map.header.frame_id = this->config.odom_frame;
    // publish point cloud.
    this->pub.publish(current_map);

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = odometry.x;
    pose.pose.position.y = odometry.y;
    pose.pose.position.z = odometry.z;
    tf2::Quaternion pose_q;
    pose_q.setRPY(euler_rot_.roll, euler_rot_.pitch, euler_rot_.yaw);

    pose.pose.orientation = tf2::toMsg(pose_q);
    pose.header.frame_id = this->config.odom_frame;
    pose.header.stamp = ros::Time::now();
    this->stampedPosePub.publish(pose);

    nav_msgs::Odometry laserOdometryROS;
    laserOdometryROS.header.frame_id = this->config.odom_frame;
    laserOdometryROS.header.stamp = ros::Time::now();
    laserOdometryROS.pose.pose = pose.pose;
    this->LaserOdometryPub.publish(laserOdometryROS);
}

int main(int argc, char **argv) {
    // Initialize ROS
    ros::init(argc, argv, "fbm_icp");

    // Create a ndt mapping object.
    fbm_icp fbm_icp_map;

    while (ros::ok()) {
        fbm_icp_map.mtx_keypoints.lock();
        int size = fbm_icp_map.keypoints_vec.size();
        fbm_icp_map.mtx_keypoints.unlock();
        for (int i = 0; i < size; i++) {
            fbm_icp_map.mtx_keypoints.lock();
            std::pair<MapPointCloud, euler_rot> key_points_pair = fbm_icp_map.keypoints_vec[0];
            fbm_icp_map.mtx_keypoints.unlock();
            clock_t begin = clock();
            fbm_icp_map.icp_mapping(key_points_pair);
            clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout << "Elapsed_time: " << elapsed_secs << " sec, Freq:" << 1 / elapsed_secs << " Hz" << std::endl;
            std::cout << "******************************************************" << std::endl;
            fbm_icp_map.mtx_keypoints.lock();
            fbm_icp_map.keypoints_vec.erase(fbm_icp_map.keypoints_vec.begin());
            fbm_icp_map.mtx_keypoints.unlock();
        }
        ros::spinOnce();
    }
}
