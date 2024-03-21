#!/usr/bin/env python

import sys
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

class ControlHusky():
    def __init__(self, name):
        self.TIMEOUT_DURATION = 10
        self.name = name
        self.client = actionlib.SimpleActionClient(f'{self.name}/move_base', MoveBaseAction)

    def send_goal(self, x, y):
        if (not self.client.wait_for_server(rospy.Duration(self.TIMEOUT_DURATION))):
            rospy.logerr('Action server timeout')
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.orientation.w = 1.0

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Destination not reachable')
            return False

        return True

    
        
if __name__ == '__main__':
    rospy.init_node('static_navigation_multiple_huskies_node')
    
    husky1 = ControlHusky("husky1")
    husky2 = ControlHusky("husky2")

    while True:
        inp1 = input("Insert goal coordinates for husky1: ")
        if inp1 == "exit": break
        x1, y1 = inp1.split()
        
        husky1.send_goal(x1, y1)

        inp2 = input("Insert goal coordinates for husky2: ")
        if inp2 == "exit": break
        x2, y2 = inp2.split()
        
        husky2.send_goal(x2, y2)


    