#!/usr/bin/env python

#import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
import actionlib

from mur_robot.msg import LoadObjectAction, LoadObjectGoal

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
        

    def execute(self,userdata):
        rospy.loginfo('Executing state moveBasetoPlace1')
        
        
        goal_target = rospy.get_param("mir/Storage1")
        # put moveBase Action Client here 
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
                        rospy.loginfo("Waiting for the move_base action server to come up")


        #goal_target= choose()
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
        rospy.sleep(20)
        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return 'arrived_at_Place_1'
        else:
            return 'failed_move_Base_to_Place_1'
        


# define state moveBasetoPlace2
class moveBasetoPlace2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived_at_Place_2','failed_move_Base_to_Place_2','aborted_move_Base_to_Place_2'])
        

    def execute(self,userdata):
        rospy.loginfo('Executing state moveBasetoPlace2')
        
        
        goal_target = rospy.get_param("mir/Storage2")
        # put moveBase Action Client here 
        ac2 = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac2.wait_for_server(rospy.Duration.from_sec(5.0))):
                        rospy.loginfo("Waiting for the move_base action server to come up")


        #goal_target= choose()
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
        ac2.send_goal(goal)
        rospy.sleep(20)
        ac2.wait_for_result(rospy.Duration(60))


        return 'arrived_at_Place_2'
    
        return 'failed_move_Base_to_Place_2'



# define state moveArm_1
class moveArm_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded_moveArm_1','failed_moveArm_1','aborted_moveArm_1'])
        

    def execute(self,userdata):
        rospy.loginfo('Executing state moveArm_1')
        

        client = actionlib.SimpleActionClient('load_object', LoadObjectAction)
        client.wait_for_server()

        goal = LoadObjectGoal()
        goal.ObjectID=int(1)
        # Fill in the goal here
        
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(45.0))

        res= client.get_result()

        return 'succeeded_moveArm_1'


# define state moveArm_2
class moveArm_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded_moveArm_2','failed_moveArm_2','aborted_moveArm_2'])
        

    def execute(self,userdata):
        rospy.loginfo('Executing state moveArm_2')
        
        client = actionlib.SimpleActionClient('load_object', LoadObjectAction)
        client.wait_for_server()

        goal = LoadObjectGoal()
        goal.ObjectID=int(1)
        # Fill in the goal here
        
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(45.0))

        res= client.get_result()

        return 'succeeded_moveArm_2'
        

# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')

        goal_target = rospy.get_param("mir/Flatscreen")
        # put moveBase Action Client here 
        ac2 = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #wait for the action server to come up
        while(not ac2.wait_for_server(rospy.Duration.from_sec(5.0))):
                        rospy.loginfo("Waiting for the move_base action server to come up")


        #goal_target= choose()
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
        ac2.send_goal(goal)
        rospy.sleep(20)
        ac2.wait_for_result(rospy.Duration(60))


        return 'outcome1'
        




def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('moveBasetoPlace1', moveBasetoPlace1(), 
                               transitions={'arrived_at_Place_1':'moveArm_1', 'aborted_move_Base_to_Place_1':'outcome4','failed_move_Base_to_Place_1':'outcome4'})

        smach.StateMachine.add('moveBasetoPlace2', moveBasetoPlace2(), 
                               transitions={'arrived_at_Place_2':'moveArm_2', 'aborted_move_Base_to_Place_2':'outcome4','failed_move_Base_to_Place_2':'outcome4'})

        smach.StateMachine.add('moveArm_1', moveArm_1(), 
                               transitions={'succeeded_moveArm_1':'moveBasetoPlace2', 'failed_moveArm_1':'outcome4','aborted_moveArm_1':'Bar'})
        
        smach.StateMachine.add('moveArm_2', moveArm_2(), 
                               transitions={'succeeded_moveArm_2':'Bar', 'failed_moveArm_2':'outcome4','aborted_moveArm_2':'Bar'})

        smach.StateMachine.add('Bar', Bar(), 
                               transitions={'outcome1':'outcome4'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()