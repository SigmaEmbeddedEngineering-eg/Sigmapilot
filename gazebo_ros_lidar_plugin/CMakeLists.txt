cmake_minimum_required(VERSION 3.0.2)
project(gazebo_ros_lidar_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gazebo_ros
  gazebo_plugins
  gazebo_msgs
  gazebo_dev
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  nav_msgs
  urdf
  pcl_ros
  cv_bridge
  image_transport
)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(XML libxml-2.0)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS thread)
 find_package(gazebo REQUIRED)
 find_package( PCL REQUIRED )

execute_process(COMMAND
  pkg-config --variable=plugindir OGRE
  OUTPUT_VARIABLE OGRE_PLUGIN_PATH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
	
  CATKIN_DEPENDS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  urdf
  gazebo_msgs
  cv_bridge
  image_transport

  DEPENDS
	gazebo SDF
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS}
  ${OGRE-Paging_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  include/gazebo_ros_lidar_plugin
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS}
  ${OGRE-Paging_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${Eigen3_LIBRARY_DIRS}
  include/gazebo_ros_lidar_plugin

)

add_library(gazebo_ros_lidar_plugin src/gazebo_ros_lidar_plugin.cpp src/custom_lidar.cpp)
target_link_libraries(gazebo_ros_lidar_plugin RayPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## Mark cpp header files for installation
install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
)
