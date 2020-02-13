#! /usr/bin/env python

import roslib
import rospy
import actionlib
from onrobot_rg2_gripper import OnRobotGripperRG2
from mur_robot.msg import RG2GripperAction, RG2GripperGoal

from ur_msgs.msg import IOStates

if __name__ == '__main__':
    rospy.init_node('rg2gripper_action_client')


    try:
        client = actionlib.SimpleActionClient('rg2gripper', RG2GripperAction)
        client.wait_for_server()
        print "Client is connected to  server"
        width=raw_input("Width: ")

        goal = RG2GripperGoal()
        goal.target_width=int(width)
        goal.target_force=int(20)
        goal.payload=int(20)
        goal.set_payload=False
        goal.depth_compensation=False
        goal.slave=False


        # Fill in the goal here
        
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(10.0))

        res= client.get_result()
        print res
    except:
        print "Could not connect to Action server"

    
    