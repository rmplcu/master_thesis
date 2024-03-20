#!/bin/bash

#Variables
gazebo_launchfile=husky_playpen.launch

cd /home/luca/Desktop/master_thesis

#Start roscore

#Launch gazebo
gnome-terminal --tab -- roslaunch husky_gazebo $gazebo_launchfile
sleep 5

#Launch rviz
gnome-terminal --tab -- roslaunch husky_viz view_robot.launch

#Map server
gnome-terminal --tab -- rosrun map_server map_server ./workspace/src/husky/maps/map.yaml

#Navigation stack
gnome-terminal --tab -- roslaunch husky_navigation amcl_demo.launch map_file:=./workspace/src/husky/maps/playpen_map.yaml

#Run my node
gnome-terminal --tab -- roscore
gnome-terminal --tab -- rosrun husky static_navigation.py