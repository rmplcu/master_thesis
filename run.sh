#Start roscore
gnome-terminal --tab -- roscore

#Launch gazebo
gnome-terminal --tab -- roslaunch husky_gazebo my_world.Launch

#Launch rviz
gnome-terminal --tab -- roslaunch 

#Run my node
rosrun husky static_navigation.py