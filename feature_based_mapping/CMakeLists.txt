cmake_minimum_required(VERSION 3.0.2)
project(feature_based_mapping)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
tf
message_filters
laser_geometry
pcl_conversions
pcl_ros
cv_bridge
image_transport
nav_msgs
)

find_package(OpenCV REQUIRED
NO_MODULE
PATHS /usr/local
NO_DEFAULT_PATH
)
message("OpenCV version: ${OpenCV_VERSION}")
message("OpenCV DIR: ${OpenCV_INCLUDE_DIRS}")
message("OpenCV LIB DIR: ${OpenCV_LIB_DIR}")

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIB_DIR})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES feature_based_mapping
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
  include/feature_based_mapping
  ${catkin_INCLUDE_DIRS}
)


add_executable(fbm_icp src/fbm_icp.cpp )
target_link_libraries(fbm_icp ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})