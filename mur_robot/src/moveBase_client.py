#! /usr/bin/env python

import roslib
#roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,  Point, Twist, Pose
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
import time
import numpy as np


def choose():
		
		#rospy.loginfo("|WHERE TO GO?")
		choice = raw_input("Where to go:")
		pose = rospy.get_param(choice)

		print pose
		return pose


if __name__ == '__main__':
    rospy.init_node('moveBase_client')

    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    #wait for the action server to come up
    while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                    rospy.loginfo("Waiting for the move_base action server to come up")


    goal_target= choose()
    goal = MoveBaseGoal()

    #set up the frame parameters
    goal.target_pose.header.frame_id = "/map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/
    goal.target_pose.pose.position.x =  goal_target[0]
    goal.target_pose.pose.position.y =  goal_target[1]
    goal.target_pose.pose.position.z =  goal_target[2]
    goal.target_pose.pose.orientation.x = goal_target[3]
    goal.target_pose.pose.orientation.y = goal_target[4]
    goal.target_pose.pose.orientation.z = goal_target[5]
    goal.target_pose.pose.orientation.w = goal_target[6]

    start = rospy.get_time()  #get the current time

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)
    
    ac.wait_for_result(rospy.Duration(60))
    