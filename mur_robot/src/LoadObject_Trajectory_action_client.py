#! /usr/bin/env python

import roslib
#roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

from mur_robot.msg import LoadObjectAction, LoadObjectGoal

if __name__ == '__main__':
    rospy.init_node('load_object_trajectory_action_client')
    client = actionlib.SimpleActionClient('load_object', LoadObjectAction)
    client.wait_for_server()

    goal = LoadObjectGoal()
    goal.ObjectID=int(1)
    # Fill in the goal here
    
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(45.0))

    res= client.get_result()
    print res
    