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

#include <algorithm>
#include <assert.h>
#include <gazebo_ros_lidar_plugin/gazebo_ros_lidar_plugin.h>
#include <string>

#include <gazebo/common/Exception.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>
#include <sdf/Param.hh>
#include <sdf/sdf.hh>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <gazebo_plugins/gazebo_ros_laser.h>
#include <ignition/math/Rand.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLiDAR);

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboRosLiDAR::GazeboRosLiDAR() {}

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboRosLiDAR::~GazeboRosLiDAR()
  {
    this->rosnode_->shutdown();
    delete this->rosnode_;
  }

  ////////////////////////////////////////////////////////////////////////////////

  // Load the plugin
  void GazeboRosLiDAR::Load(sensors::SensorPtr _parent,
                            sdf::ElementPtr _sdf)
  {

    // load plugin
    RayPlugin::Load(_parent, this->sdf);
    // Get the world name.

    std::string worldName = _parent->WorldName();
    this->world_ = physics::get_world(worldName);
    // save pointers
    this->sdf = _sdf;

    GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
    this->parent_ray_sensor_ = dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if (!this->parent_ray_sensor_)
      ROS_INFO_NAMED("LiDAR", "GazeboRosLiDAR controller "
                              "requires a Ray Sensor as its parent");

    this->robot_namespace_ = GetRobotNamespace(_parent, _sdf, "Laser");

    if (!this->sdf->HasElement("frameName"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <frameName>, defaults to /world");
      this->frame_name_ = "/world";
    }
    else
      this->frame_name_ = this->sdf->Get<std::string>("frameName");

    if (!this->sdf->HasElement("topicName"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <topicName>, defaults to /world");
      this->topic_name_ = "/world";
    }
    else
      this->topic_name_ = this->sdf->Get<std::string>("topicName");

    // Get sensor position
    std::string sensor_pose;
    if (!this->sdf->HasElement("pose"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <pose>, defaults to <0, 0, 0, 0, 0, 0>");
      sensor_pose = "0 0 0 0 0 0";
    }
    else
      sensor_pose = this->sdf->Get<std::string>("pose");
    this->custom_lidar_.LP.mp.Set(sensor_pose);
    ROS_INFO_NAMED("LiDAR", "x:%f, y:%f,z:%f,roll:%f,pitch:%f,yaw:%f",
                   this->custom_lidar_.LP.mp.x, this->custom_lidar_.LP.mp.y, this->custom_lidar_.LP.mp.z,
                   this->custom_lidar_.LP.mp.roll, this->custom_lidar_.LP.mp.pitch, this->custom_lidar_.LP.mp.yaw);

    if (!this->sdf->HasElement("cam_topic_name"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <cam_topic_name>, fusion disabled");
      this->cam_topic_name = "";
      this->custom_lidar_.LP.enable_fusion = false;
    }
    else
    {
      this->cam_topic_name = this->sdf->Get<std::string>("cam_topic_name");
      this->custom_lidar_.LP.enable_fusion = true;
    }

    if (!this->sdf->HasElement("cam_fov"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <cam_fov>, defaults to 0.0001");
      this->custom_lidar_.LP.cam_fov = 0.0001;
    }
    else
    {
      std::string s_cam_fov = this->sdf->Get<std::string>("cam_fov");
      this->custom_lidar_.LP.cam_fov = std::stof(s_cam_fov);
    }

    if (!this->sdf->HasElement("sensorId"))
    {
      ROS_INFO_NAMED("LiDAR",
                     "Laser plugin missing <sensorId>, defaults to 0");
      this->sensor_id = "0";
    }
    else
    {
      this->sensor_id = this->sdf->Get<std::string>("sensorId");
    }

    this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED(
          "LiDAR",
          "A ROS node for Gazebo has not been initialized, unable to load "
          "plugin. "
              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
                 "the gazebo_ros package)");
      return;
    }

    ROS_INFO_NAMED("LiDAR", "Starting Laser Plugin (ns = %s)",
                   this->robot_namespace_.c_str());
    // ros callback queue for processing subscription
    this->deferred_load_thread_ =
        boost::thread(boost::bind(&GazeboRosLiDAR::LoadThread, this));
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Load the controller
  void GazeboRosLiDAR::LoadThread()
  {
    this->gazebo_node_ =
        gazebo::transport::NodePtr(new gazebo::transport::Node());
    this->gazebo_node_->Init(this->world_name_);

    this->pmq.startServiceThread();

    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
    if (this->tf_prefix_.empty())
    {
      this->tf_prefix_ = this->robot_namespace_;
      boost::trim_right_if(this->tf_prefix_, boost::is_any_of("/"));
    }
    ROS_INFO_NAMED("LiDAR",
                   "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
                   this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

    // resolve tf prefix
    this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

    if (this->topic_name_ != "")
    {
      ros::AdvertiseOptions pcl_ao = ros::AdvertiseOptions::create<PointCloud>(
          "LiDAR" + this->sensor_id + "/pcl", 1000,
          boost::bind(&GazeboRosLiDAR::LaserConnect, this),
          boost::bind(&GazeboRosLiDAR::LaserDisconnect, this),
          ros::VoidPtr(), NULL);
      this->pcl_pub_ = this->rosnode_->advertise(pcl_ao);
      this->pcl_pub_queue_ = this->pmq.addPub<PointCloud>();
    }

    // sensor generation off by default
    this->parent_ray_sensor_->SetActive(false);

    this->it_ = new image_transport::ImageTransport(*this->rosnode_);
    // Subscrive to input video feed and publish output video feed
    image_sub_ = this->it_->subscribe(this->cam_topic_name, 1,
                                      &GazeboRosLiDAR::imageCb, this);
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Camera callback
  void GazeboRosLiDAR::imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      mtx.lock();
      this->cam_feed = cv_ptr->image.clone();
      mtx.unlock();
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  ////////////////////////////////////////////////////////////////////////////////
  // Increment count
  void GazeboRosLiDAR::LaserConnect()
  {
    this->laser_connect_count_++;
    if (this->laser_connect_count_ == 1)
      this->laser_scan_sub_ =
          this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                        &GazeboRosLiDAR::OnScan, this);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Decrement count
  void GazeboRosLiDAR::LaserDisconnect()
  {
    this->laser_connect_count_--;
    if (this->laser_connect_count_ == 0)
      this->laser_scan_sub_.reset();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Convert new Gazebo message to ROS message and publish it
  void GazeboRosLiDAR::OnScan(ConstLaserScanStampedPtr &_msg)
  {
    this->custom_lidar_.LP.min_horizontal_angle = _msg->scan().angle_min();
    this->custom_lidar_.LP.max_horizontal_angle = _msg->scan().angle_max();
    this->custom_lidar_.LP.horizontal_step_angle = _msg->scan().angle_step();

    this->custom_lidar_.LP.min_vertical_angle = _msg->scan().vertical_angle_min();
    this->custom_lidar_.LP.max_vertical_angle = _msg->scan().vertical_angle_max();
    this->custom_lidar_.LP.vertical_step_angle = _msg->scan().vertical_angle_step();

    this->custom_lidar_.LP.vertical_points_count = _msg->scan().vertical_count();
    this->custom_lidar_.LP.horizontal_points_count = _msg->scan().count();

    if (!this->custom_lidar_.LP.parameters_initialized)
    {
      this->custom_lidar_.init_lidar_params();
    }

    // copy the ranges of distances from msg data containers, to a vector.
    std::vector<float> ranges;
    ranges.resize(_msg->scan().ranges_size());
    std::copy(_msg->scan().ranges().begin(), _msg->scan().ranges().end(),
              ranges.begin());

    mtx.lock();
    cv::Mat cam_feed_ = this->cam_feed.clone();
    mtx.unlock();

    // Create a PCL_msg
    PointCloud pcl_msg;
    pcl_msg.header.frame_id = this->frame_name_;
    pcl_conversions::toPCL(ros::Time::now(), pcl_msg.header.stamp);
    pcl_msg.height = this->custom_lidar_.LP.vertical_points_count;
    pcl_msg.width = this->custom_lidar_.LP.horizontal_points_count;
    this->custom_lidar_.construct_lidar_frame(ranges, cam_feed_, &pcl_msg);

    this->pcl_pub_queue_->push(pcl_msg, this->pcl_pub_);

  }
} // namespace gazebo
