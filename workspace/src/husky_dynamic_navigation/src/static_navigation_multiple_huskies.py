#! /usr/bin/env python

import rospy 
from workspace.src.husky_dynamic_navigation.src.husky_dynamic_navigation.control_husky import ControlHusky

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


    