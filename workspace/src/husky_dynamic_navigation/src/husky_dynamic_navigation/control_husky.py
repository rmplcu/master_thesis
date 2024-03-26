#! /usr/bin/ python

from turtle import pos
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose

class ControlHusky():
    def __init__(self, name):
        self.TIMEOUT_DURATION = 10
        self.__name = name
        self.__current_pose = None
        self.__client = actionlib.SimpleActionClient(f'{self.__name}/move_base', MoveBaseAction)

        def pose_callback(pose):
            self.__current_pose = pose.pose

        rospy.Subscriber(f'{self.__name}/amcl_pose', PoseWithCovarianceStamped, pose_callback, queue_size=10)

    def send_goal(self, x=None, y=None, w=None, pose=None):
        if pose is None and (x is None or y is None): raise ValueError("None parameter")

        if (not self.__client.wait_for_server(rospy.Duration(self.TIMEOUT_DURATION))):
            rospy.logerr('Action server timeout')
            return False

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
    
        if pose is not None:
            goal.target_pose.pose = pose.pose
        else:
            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = (w is None and 1) or w

        self.__client.send_goal(goal)
        self.__client.wait_for_result()

        if self.__client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr('Destination not reachable')
            return False

        rospy.loginfo('Destination reached!')
        return True


    def get_current_pose(self):
        if self.__current_pose is None:
            rospy.logwarn(f"Current position not set for {self.__name}")

        return self.__current_pose