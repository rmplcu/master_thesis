#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


#Returns true if position x, y is reached, false otherwise
# x: x coordinate position
# y: y coordinate position 
def move_robot(x, y, name):
    
    client = actionlib.SimpleActionClient(f'/{name}/move_base', MoveBaseAction)
    print(f'{name}/move_base')

    if (not client.wait_for_server(rospy.Duration(10))):
        rospy.logerr('Action server timeout')
        return False

    #Set goal position
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = f'{name}_tf/frame_id'
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
    rospy.init_node('static_navigation_multiple_huskies_node')

    try :
        while True:
            try :
                x, y, name = input('Insert coordinates and robot: ').split()
                move_robot(x,y,name)    
            except:
                print('Insert correct parameters')
    except KeyboardInterrupt:
        pass



    