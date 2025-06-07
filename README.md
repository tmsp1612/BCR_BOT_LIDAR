
# BCR BOT Robot adapted with 3d LiDAR and RGB camera



Adaptation of the BCR BOT mobile robot, whose original code is available at https://github.com/blackcoffeerobotics/bcr_bot, with a 3D LiDAR and a RGB camera (and in this way complement the depth camera)

The LiDAR import the mesh of the Velodyne VLP16/32.

Tested in ROS2 Humble on ubuntu 22.04

This version only supports the simulation mode on Gazebo (gz) Fortress. So make sure you have gz installed (https://gazebosim.org/docs/latest/ros_installation/)


## Dependencies

First is need to download the repo related to the meshes of 3D LiDAR (Velodyne)
The meshes can be found on this repo adapted to some ROS2 Humble troubles: [hee](https://github.com/tmsp1612/Velodyne_Simulator_Humble)



## RUN

To launch the robot in gz,
```bash
ros2 launch bcr_bot gz.launch.py
```




