cmake_minimum_required(VERSION 3.0.2)
project(laser_scans_fusion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_filters
  laser_geometry
  pcl_conversions
  pcl_ros
  tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES laser_scans_fusion
  CATKIN_DEPENDS roscpp rospy std_msgs message_filters laser_geometry pcl_conversions pcl_ros
)

include_directories(
  include
  include/laser_scans_fusion
  ${catkin_INCLUDE_DIRS}
)


add_executable(laser_scans_fusion src/laser_scans_fusion.cpp src/scan_fusion.cpp)
target_link_libraries(laser_scans_fusion ${catkin_LIBRARIES})