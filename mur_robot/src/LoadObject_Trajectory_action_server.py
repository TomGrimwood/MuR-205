#! /usr/bin/env python

import roslib
import rospy
import actionlib
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg
from mur_robot.msg import LoadObjectAction, LoadObjectResult, LoadObjectFeedback

class LoadObjectServer:
  _result=LoadObjectResult()
  _feedback=LoadObjectFeedback()

  def __init__(self):
    
    self.server = actionlib.SimpleActionServer('load_object', LoadObjectAction, self.execute, False)
    self.pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    
    
    
    self.server.start()
    print "Action Server started. Waiting for Action Client to call..."

  def execute(self, goal):
    print "Moving Arm to: ",goal 
    #self._feedback.Progress = int(23)
    #self.server.publish_feedback(self._feedback)

    msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
    
    print msg.is_program_running

    #var=raw_input("Go on: ")

    

    self.pub.publish('movej('+str(rospy.get_param("arm/travel_pose_j"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
      
    print "Move1"
      

    
    self.pub.publish('movej('+str(rospy.get_param("arm/right_above_j"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
    print "Move2"
    
    #self._feedback.Progress = int(89)
    #self.server.publish_feedback(self._feedback)
    
    
    
    
    self.pub.publish('movel(p'+str(rospy.get_param("arm/right_tcp"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
    print "Move3"
    
    
    
    
    self.pub.publish('movel(p'+str(rospy.get_param("arm/right_above_tcp"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
    print "Move4"


    
    self.pub.publish('movej('+str(rospy.get_param("arm/travel_pose_j"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
    print "Move5"
    
    
    
    

    print "Trajectory Done"
    self._result.Success=int(100)
    self.server.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('load_object_action_server')
  server = LoadObjectServer()
  rospy.spin()