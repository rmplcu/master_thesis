#!/bin/bash

#Variables
roslaunch husky_gazebo husky_playpen.launch

roslaunch husky_viz view_robot.launch

roslaunch husky_navigation gmapping_demo.launch