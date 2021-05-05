![CI](https://github.com/SigmaEmbeddedEngineering/Sigmapilot/workflows/CI/badge.svg?branch=main)

# Sigmapilot
Sigmpilot is an online demo for Sigma embedded engineering clients representing our skills in autonomous robotics, sensor fusion and simulation environments.
Sigmapilot is an all-in-one open-source software for autonomous UGVs and industrial robots. the target industries that Sigmapilot is working on are farming industry, mining industry, warehouses industry, and construction industry. 

we were always wondering why there isnt any framework that processes robust autonomous software stack on **Raspberry Pi** board that can be used in UGV industries, do we always need a **NVIDIA** board?. The result was **Sigmapilot** :)

# Sigmapilot Featues

- Modular software components can work with different sensors(benwake/velodyne)
- 3D semantic segmentaiton object detection.
- Freespace estimation.
- Integration of Sigmapilot with ROS packages such as gmapping, move base, and amcl.
- Free space fusion for cocoon Setup.
- 3D point cloud assembly.

# Sigmapilot Planned Features

- 3D mapping and matching.
- Object tracking.
- new approaches for 2D mapping and matching.
- DNN detection and semantic segmentation.
- Camera-LiDAR fusion.


# Demos:
## Navigation
### Sigmapilot Farm simulation demo Velodyne 

    roslaunch gazebo_simulator robot_navigation.launch sensor:=velodyne world_name:='$(find gazebo_simulator)/worlds/turtlebot3_world.world' map_file:='$(find gazebo_simulator)/maps/map.yaml'

![velo_farm](gazebo_simulator/docs/velodyne_farming_demo.gif)

### Sigmapilot Cave simulation demo velodyne

    roslaunch gazebo_simulator robot_navigation.launch sensor:=velodyne world_name:='$(find gazebo_simulator)/worlds/50m_long_mine_world.world' map_file:='$(find gazebo_simulator)/maps/mymap_cave.yaml'

![velo_cave](gazebo_simulator/docs/velodyne_cave.gif)

### Sigmapilot Farm simulation demo Benwake cocoon

    roslaunch gazebo_simulator robot_navigation.launch sensor:=benwake world_name:='$(find gazebo_simulator)/worlds/turtlebot3_world.world' map_file:='$(find gazebo_simulator)/maps/map.yaml'

![benwake_farm](gazebo_simulator/docs/benwake_farming_demo.gif)

## SLAM

### Sigmapilot map farm environment

    roslaunch gazebo_simulator Sigmapilot_gazebo.launch sensor:=velodyne

![benwake_farm](gazebo_simulator/docs/slam_farm.gif)


### Sigmapilot map cave environment

    roslaunch gazebo_simulator Sigmapilot_gazebo.launch sensor:=velodyne world_name:='$(find gazebo_simulator)/worlds/50m_long_mine_world.world' 

![benwake_farm](gazebo_simulator/docs/slam_cave.gif)

## Raspberry pi benchmarking

 follow the instructions in [sgm_lidar_clustering](sgm_lidar_clustering/README.md)

## SGM clustering Vs lidar_euclidean_cluster_detect

 read the benchmarking section in [sgm_lidar_clustering](sgm_lidar_clustering/README.md)


## Partners
![sigma](https://media-exp1.licdn.com/dms/image/C5112AQE5H7TaTAI58g/article-cover_image-shrink_600_2000/0/1520217198254?e=1628121600&v=beta&t=-vJWqvr0H6QSOoZ02t0429B_iRo2B3aHw01mcCpx1bQ)


## [Contact us]()
