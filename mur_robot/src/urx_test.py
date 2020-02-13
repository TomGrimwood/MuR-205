#!/usr/bin/env python
#import sys
import urx
#import logging
from onrobot_rg2_gripper import OnRobotGripperRG2

if __name__ == '__main__':


	rob = urx.Robot("192.168.12.90")
	gripper = OnRobotGripperRG2(rob)
	
	#try:
	print "0"
	gripper.open_gripper(target_width=0)
	#rob.send_program(robotiqgrip.ret_program_to_run())
	
	print "1"

	var=raw_input("go on: ")
	#except:
	#pass

	#try:
	print "2"
	gripper.open_gripper(target_width=100)

	print "3"
	#except:
	#pass

	rob.close()
	