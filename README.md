# Master Thesis Project: autonomous navigation of mobile robots in narrow corridors.

## Author
Luca Rompani: 1/03/2024 - current

## Description

This project focus on the navigation of two husky robots in an environment characterized by a narrow corridor, very common in many warehouses.

The adopted strategy is based on priority. the robot with a lower level of priority will wait for the other robot to pass through the corridor bofore starting traversing it. In this way the robots won't get stuck inside the corridor, too narrow to fit both.

## How to use

To run the program use **husky_dynamic_navigation.launhc**: this is the main launch file and it's used to spawn the robots and their move_bases and amcls in a world.

It is possible to specify some parameters: 

- **world_name**: the name of the world to spawn the robots in. Can be chosen between *playpen* (a general outdoor scenario), *narrow_corridor* (a world with a narrow corridor), *narrow_corridors* (a world with two narrow corridors).

- **h1_base_local_planner** (and/or h2_base_local_planner): the name of the local planner to use.

- **h1_base_local_planner_package** (and/or h2_base_local_planner_package): the name of the package the local planner belongs to.

- **map_file**: it is possible to use a custom map file. If not set, the one corresponding to the correct world will be automatically selected.

**WARNING**
It is advisable to use this following command to run the project:

```roslaunch husky_dynamic_navigation husky_dynamic_navigation.launch world_name:=narrow_corridor```

The default local planner used is **SimpleMPCLocalPlanner**, a custom local planner studied for the narrow corridor environment. It may not work properly if the parameters in the files [SimpleMPCLocalPlanner_husky1.yaml](workspace/src/husky_dynamic_navigation/config/SimpleMPCLocalPlanner_husky1.yaml) and [SimpleMPCLocalPlanner_husky2.yaml](workspace/src/husky_dynamic_navigation/config/SimpleMPCLocalPlanner_husky2.yaml) are not setup correctly and the launch file is used with different parameters.

## Dependencies

This package depends on:

- [ros_noetic](http://wiki.ros.org/noetic)
- [ros husky](http://wiki.ros.org/Robots/Husky)
- [mpc_local_planner](http://wiki.ros.org/mpc_local_planner) (mpc_local_planner_msg, mpc_local_planner_examples)
- [gazebo_ros_2DMap_plugin](https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin)
- [spatio_temporal_voxel_layer](http://wiki.ros.org/spatio_temporal_voxel_layer)
- [ira_laser_tools](http://wiki.ros.org/ira_laser_tools)
- [pointcloud_to_laserscan](https://wiki.ros.org/pointcloud_to_laserscan)

## Deatiled information

For more information:

- regarding *husky_dynamic_navigation* package, click [here](workspace/src/husky_dynamic_navigation/README.md).
- regarding *simple_mpc_local_planner* pacakge, click [here](workspace/src/simple_mpc_local_planner/README.md).
