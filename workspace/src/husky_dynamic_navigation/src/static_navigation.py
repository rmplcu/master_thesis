#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


#Returns true if position x, y is reached, false otherwise
# x: x coordinate position
# y: y coordinate position 
def move_robot(x, y):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    
    if (not client.wait_for_server(rospy.Duration(10))):
        rospy.logerr('Action server timeout')
        return False

    #Set goal position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1
    goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = float(x), float(y)

    client.send_goal(goal)
    client.wait_for_result()

    if client.get_state() != GoalStatus.SUCCEEDED:
        rospy.logerr("Destination not reached")
        return False
    
    rospy.loginfo("Destination reached")
    return True



if __name__ == '__main__':
    rospy.init_node('static_navigation_node')

    try :
        while True:
            x, y = input('Insert coordinates: ').split()
            move_robot(x,y)
        
    except KeyboardInterrupt:
        pass



    