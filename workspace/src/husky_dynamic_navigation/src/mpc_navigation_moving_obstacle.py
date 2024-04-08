#!/usr/bin/env python

from turtle import pos
import rospy
from husky_dynamic_navigation.control_husky import ControlHusky
import threading
from geometry_msgs.msg import PoseWithCovariance

TIMEOUT = 10 #seconds

def run():
    rospy.init_node('MPC_moving_obstacle')
    
    h2 = ControlHusky("husky2")

    goal_pose = PoseWithCovariance()
    goal_pose.pose.position.x = 8.0
    goal_pose.pose.position.y = 3.0
    goal_pose.pose.orientation.w = 1.0

    h2.send_goal(pose=goal_pose)



if __name__ == '__main__':
    run()