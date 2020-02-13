#! /usr/bin/env python

import roslib
import rospy
import urx
import actionlib
from std_msgs.msg import String
from onrobot_rg2_gripper import OnRobotGripperRG2
from mur_robot.msg import RG2GripperAction, RG2GripperResult, RG2GripperFeedback
from ur_msgs.msg import IOStates

class RG2GripperActionServer:
	_result=RG2GripperResult()
	_feedback=RG2GripperFeedback()

	def __init__(self):
		
		self.server = actionlib.SimpleActionServer('rg2gripper', RG2GripperAction, self.execute, False)
		
		
		
		
		self.server.start()
		print "RG2Gripper Action Server started. Waiting for Action Client to call..."

	def execute(self, goal):
		print "Received Goal: ",goal 

		self._feedback.Progress = int(1)
		self.server.publish_feedback(self._feedback)
		#Load URX Driver and initialize RG2-Conncetion
		try:
			robot_ip=rospy.get_param("/ur_driver/robot_ip_address")
			rob = urx.Robot(robot_ip)
			gripper = OnRobotGripperRG2(rob)
			self._feedback.Progress = int(23)
			self.server.publish_feedback(self._feedback)
		except:
			print "Failed to Load urx_driver"
			self.server.set_aborted()
			#pass
		



		try:
			gripper.open_gripper(target_width=goal.target_width)
			#rob.send_program(robotiqgrip.ret_program_to_run())

			self._feedback.Progress = int(49)
			self.server.publish_feedback(self._feedback)
		except:
			print "Failed to send gripping program to robot"
			#pass
		

		rospy.sleep(0.2)
		
		is_grippper_waiting=False
		while is_grippper_waiting==False:
			rospy.sleep(0.1)
			io_states=rospy.wait_for_message('/ur_driver/io_states',IOStates)
			is_grippper_waiting= io_states.digital_out_states[16].state
			print is_grippper_waiting

			'''
			gripping_contact_force= io_states.digital_out_states[17].state
			if gripping_contact_force==True:
				# Gripper contact: setting success as result
				rob.close()
				self._feedback.Progress = int(100)
				print "Contact was made. Gripper Done"
				self._result.Success=int(100)
				self.server.set_succeeded(self._result)
				
				break

		print "Grip completed"
		

		if io_states.digital_out_states[17].state ==False:
			print "No Contact Force detected."
		'''
		
		


		rob.close()
		self._feedback.Progress = int(100)
		self.server.publish_feedback(self._feedback)

		print "Gripper Done"
		self._result.Success=int(100)
		self.server.set_succeeded(self._result)


if __name__ == '__main__':
	rospy.init_node('rg2gripper_action_server')
	server = RG2GripperActionServer()
	rospy.spin()