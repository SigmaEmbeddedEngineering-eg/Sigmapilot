cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_assembler)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
tf
message_filters
laser_geometry
pcl_conversions
pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES point_cloud_assembler
  CATKIN_DEPENDS roscpp rospy std_msgs message_filters laser_geometry pcl_conversions pcl_ros
)

include_directories(
  include/point_cloud_assembler
  ${catkin_INCLUDE_DIRS}
)


add_executable(point_cloud_assembler src/point_cloud_assembler.cpp )
target_link_libraries(point_cloud_assembler ${catkin_LIBRARIES})