cmake_minimum_required(VERSION 3.0.2)
project(sgm_lidar_clustering)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  pcl_ros
  cv_bridge
  image_transport
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


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES sgm_lidar_clustering
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  include/sgm_lidar_clustering
  ${catkin_INCLUDE_DIRS}
)

add_executable(sgm_lidar_clustering src/sgm_lidar_clustering.cpp src/ground_segmentation.cpp src/sgm_segmentation.cpp)
target_link_libraries(sgm_lidar_clustering ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})