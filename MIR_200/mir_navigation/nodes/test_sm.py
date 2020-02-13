#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped,  Point, Twist, Pose
#from geometry_msgs.msg import TwistWithCovarianceStamped, TwistStamped, TwistWithCovariance
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import sys
import time
import numpy as np

# define state moveBasetoPlace1
class moveBasetoPlace1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_Place_1','failed_move_Base_to_Place_1','aborted_move_Base_to_Place_1'])
        

    def execute(self):
        rospy.loginfo('Executing state moveBasetoPlace1')
        
        try:
            # put moveBase Action Client here 

            return 'arrived_at_Place_1
        except:
            return 'failed_move_Base_to_Place_1'


# define state moveBasetoPlace2
class moveBasetoPlace2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_Place_2','failed_move_Base_to_Place_2','aborted_move_Base_to_Place_2'])
        

    def execute(self):
        rospy.loginfo('Executing state moveBasetoPlace2')
        
        try:
            # put moveBase Action Client here 

            return 'arrived_at_Place_2
        except:
            return 'failed_drive_Base_to_Place_2'



# define state moveArm_1
class moveArm_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded_moveArm_1','failed_moveArm_1','aborted_moveArm_1'])
        

    def execute(self):
        rospy.loginfo('Executing state moveArm_1')
        
        try:
            # put manipulation code here 

            return 'arrived_at_Place_1
        except:
            return 'failed_drive_Base_to_Place_1'

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(), 
                               transitions={'outcome1':'BAR', 'outcome2':'outcome4'})
        smach.StateMachine.add('BAR', Bar(), 
                               transitions={'outcome1':'FOO'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()