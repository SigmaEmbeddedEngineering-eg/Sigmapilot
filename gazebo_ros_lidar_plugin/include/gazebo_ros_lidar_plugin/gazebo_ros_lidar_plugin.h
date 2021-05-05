/*
 * Desc: LiDAR sensor ROS Gazebo plugin.
 * Author: khaled elmadawi
 * mail: khalid.elmadawi@robotion.ai
 * Date: 5 September 2020
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

#ifndef GAZEBO_ROS_LIDAR_HH
#define GAZEBO_ROS_LIDAR_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/advertise_options.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sdf/Param.hh>

#include <gazebo_plugins/PubQueue.h>

#include "custom_lidar.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <mutex>          // std::mutex
#include <boost/foreach.hpp>

namespace gazebo {

class GazeboRosLiDAR : public RayPlugin {
  /// \brief Constructor
public:
  GazeboRosLiDAR();

  /// \brief Destructor
public:
  virtual ~GazeboRosLiDAR();

  /// \brief Load the plugin
  /// \param take in SDF root element
public:
  void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Keep track of number of connctions
private:
  int laser_connect_count_;

private:
  void LaserConnect();

private:
  void LaserDisconnect();

  // Pointer to the model
  GazeboRosPtr gazebo_ros_;

private:
  std::string world_name_;

private:
  physics::WorldPtr world_;
  /// \brief The parent sensor
private:
  sensors::RaySensorPtr parent_ray_sensor_;

  /// \brief pointer to ros node
private:
  ros::NodeHandle *rosnode_;

private:
  ros::Publisher pcl_pub_;

private:
  PubQueue<PointCloud>::Ptr pcl_pub_queue_;

  /// \brief topic name
private:
  std::string topic_name_;

  /// \brief frame transform name, should match link name
private:
  std::string frame_name_;

  /// \brief tf prefix
private:
  std::string tf_prefix_;

  /// \brief for setting ROS name space
private:
  std::string robot_namespace_;

  // deferred load in case ros is blocking
private:
  sdf::ElementPtr sdf;

private:
  void LoadThread();

private:
  void imageCb(const sensor_msgs::ImageConstPtr &msg);

private:
  boost::thread deferred_load_thread_;

private:
  gazebo::transport::NodePtr gazebo_node_;

private:
  gazebo::transport::SubscriberPtr laser_scan_sub_;

private:
  void OnScan(ConstLaserScanStampedPtr &_msg);


private:
  PubMultiQueue pmq;
  std::string sensor_id;

  image_transport::ImageTransport *it_;
  image_transport::Subscriber image_sub_;
  cv::Mat cam_feed;

  CustomLidar custom_lidar_;

  std::mutex mtx;
  std::string cam_topic_name; 
};

} // namespace gazebo

#endif
