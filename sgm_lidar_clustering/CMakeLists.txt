cmake_minimum_required(VERSION 3.0.2)
project(sgm_lidar_clustering)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_definitions(-Wall -g)

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
  LIBRARIES sgm_lidar_clustering
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  include/sgm_lidar_clustering
  ${catkin_INCLUDE_DIRS}
)

add_library(sgm_clustering_lib
	src/ground_segmentation.cpp
  src/sgm_segmentation.cpp
)
target_link_libraries(sgm_clustering_lib ${catkin_LIBRARIES})

add_executable(sgm_lidar_clustering nodes/sgm_lidar_clustering.cpp)
target_link_libraries(sgm_lidar_clustering ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sgm_clustering_lib)

install(TARGETS sgm_clustering_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(sgm-test launch/gtest.test tests/sgm_unittests.cpp)
  target_link_libraries(sgm-test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} sgm_clustering_lib)
endif()