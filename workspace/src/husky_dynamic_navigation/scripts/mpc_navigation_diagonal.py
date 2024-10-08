#!/usr/bin/env python

import rospy
from husky_dynamic_navigation.control_husky import ControlHusky
import threading
from geometry_msgs.msg import PoseWithCovariance

TIMEOUT = 10 #seconds

def run():
    rospy.init_node('MPC_diagonal')
    
    h1 = ControlHusky("husky1")
    h2 = ControlHusky("husky2")

    starting_position_h1:PoseWithCovariance = None
    starting_position_h2:PoseWithCovariance = None

    start = rospy.Time.now()
    
    while (starting_position_h1 is None or starting_position_h2 is None) and (rospy.Time.now() - start) < rospy.Duration(TIMEOUT):
        starting_position_h1 = h1.get_current_pose()
        starting_position_h2 = h2.get_current_pose()

        rospy.sleep(1)

    if starting_position_h1 is None or starting_position_h2 is None:
        rospy.logerr("Starting position(s) None")
        return

    starting_position_h1.pose.position.y += 1
    starting_position_h2.pose.position.y += 1


    t1 = threading.Thread(target=lambda x: h1.send_goal(pose=x), args=(starting_position_h2,))
    t2 = threading.Thread(target=lambda x: h2.send_goal(pose=x), args=(starting_position_h1,))

    t1.start()
    t2.start()

    t1.join()
    t2.join()


if __name__ == '__main__':
    run()