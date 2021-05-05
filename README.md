# Autobotware
Autobotware is an all-in-one open-source software for autonomous UGVs and industrial robots. the target industries that Autobotware is working on are farming industry, mining industry, warehouses industry, and construction industry. 

we were always wondering why there isnt any framework that processes robust autonomous software stack on **Raspberry Pi** board that can be used in UGV industries, do we always need a **NVIDIA** board?. The result was **Aytobotware** :)

# Autobotware Featues

- Modular software components can work with different sensors(benwake/velodyne)
- 3D semantic segmentaiton object detection.
- Freespace estimation.
- Integration of Autobotware with ROS packages such as gmapping, move base, and amcl.
- Free space fusion for cocoon Setup.
- 3D point cloud assembly.

# Autobotware Planned Features

- 3D mapping and matching.
- Object tracking.
- new approaches for 2D mapping and matching.
- DNN detection and semantic segmentation.
- Camera-LiDAR fusion.


# Demos:
## Navigation
### Autobotware Farm simulation demo Velodyne 

    roslaunch autobotware_simulator robot_navigation.launch sensor:=velodyne world_name:='$(find autobotware_simulator)/worlds/turtlebot3_world.world' map_file:='$(find autobotware_simulator)/maps/map.yaml'

![velo_farm](autobotware_simulator/docs/velodyne_farming_demo.gif)

### Autobotware Cave simulation demo velodyne

    roslaunch autobotware_simulator robot_navigation.launch sensor:=velodyne world_name:='$(find autobotware_simulator)/worlds/50m_long_mine_world.world' map_file:='$(find autobotware_simulator)/maps/mymap_cave.yaml'

![velo_cave](autobotware_simulator/docs/velodyne_cave.gif)

### Autobotware Farm simulation demo Benwake cocoon

    roslaunch autobotware_simulator robot_navigation.launch sensor:=benwake world_name:='$(find autobotware_simulator)/worlds/turtlebot3_world.world' map_file:='$(find autobotware_simulator)/maps/map.yaml'

![benwake_farm](autobotware_simulator/docs/benwake_farming_demo.gif)

## SLAM

### Autobotware map farm environment

    roslaunch autobotware_simulator autobotware_gazebo.launch sensor:=velodyne

![benwake_farm](autobotware_simulator/docs/slam_farm.gif)


### Autobotware map cave environment

    roslaunch autobotware_simulator autobotware_gazebo.launch sensor:=velodyne world_name:='$(find autobotware_simulator)/worlds/50m_long_mine_world.world' 

![benwake_farm](autobotware_simulator/docs/slam_cave.gif)

## Raspberry pi benchmarking

 follow the instructions in [sgm_lidar_clustering](sgm_lidar_clustering/README.md)

