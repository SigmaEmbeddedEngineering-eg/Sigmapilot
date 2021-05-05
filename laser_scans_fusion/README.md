# Laser Scan fusion

Laser Scan fusion is a package that fuses Freespace detections in a cocoon sensor setup(multiple sensor covering different FOVs), to a **fused frame of reference** that is set in the launch file.

[reference of the idea](https://github.com/ros-perception/slam_gmapping/issues/17#issuecomment-66199895)

![Freespace_fusion](https://cloud.githubusercontent.com/assets/3719094/5348469/e31fcbd8-7efd-11e4-8630-10fbe25fa361.jpeg)

# Features:

- [x] Fuses multiple sensors Covering multiple FOV and publish it to fused sensor_msgs::LaserScan.

# Requirements:
Required packages:

* PCL
* tf

# Build: 

        caktin build laser_scans_fusion

# RUN:

clone and build **gazebo_ros_package** and **Autobotware Gazebo Ros LiDAR Plugin**.

                roslaunch laser_scans_fusion multi_scan_fusion.launch 

