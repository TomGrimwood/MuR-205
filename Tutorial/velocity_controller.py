#!/usr/bin/env python

import rospy
import tf
import time
import numpy as np
import math
from std_msgs.msg import String

def controller():
	
	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
	listener=tf.TransformListener()
	
	var=raw_input("Go on: ")
	rospy.set_param('break',1)
	while rospy.get_param('break')==1:
		rospy.sleep(0.08)
		#print "time:",time.time()

		speedy=math.cos(time.time()*3)/12 
		speedx=math.sin(time.time()*3)/10 *0
		speedg=math.sin(time.time()*3)/1 
		
		speed_vector=[speedx,speedy,0]

		

		
		#listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))
		#(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
		#euler_angles=tf.transformations.euler_from_matrix(R[:-1,:-1],axes='sxyz')

		listener.waitForTransform('/tool0_controller','/base',rospy.Time(), rospy.Duration(5))
		(trans,rot_quat) = listener.lookupTransform('/tool0_controller','/base', rospy.Time(0))

		R =	URBase_T_tcp=tf.transformations.quaternion_matrix(rot_quat)
		print R[:-1,:-1]

		euler_angles=tf.transformations.euler_from_matrix(R[:-1,:-1],axes='sxyz')
		print math.degrees(euler_angles[2])

		rotated_speeds_urbaseKS=np.dot(R[:-1,:-1],speed_vector)

		
		r_speedx=rotated_speeds_urbaseKS[0]
		r_speedy=rotated_speeds_urbaseKS[1]
		
		
		
		
		
		
		print speed_vector[0],speed_vector[1]
		print rotated_speeds_urbaseKS[0],rotated_speeds_urbaseKS[1]
		#print r_speedx,r_speedy

		
		rotated_speeds_urbaseKS[0]=0
		#rospy.loginfo(command)
	
		pub.publish('speedl(['+str(speedx)+','+str(speedy)+',0,0,0,'+str(speedg)+'], 0.5, 0.5)')

	
	
	pub.publish('Stopj(2)')
	

	#rostopic pub /ur_driver/URScript std_msgs/String "data: 'movej([3.1936185359954834, -0.9609182516681116, -2.219588104878561, -1.5329936186419886, 1.5752031803131104, 1.4252062320709229], a=1.4, v=1.05, t=0, r=0)'"





if __name__ == '__main__':
	rospy.init_node('velocity_controller_node', anonymous=True)
	try:
		controller()
	except rospy.ROSInterruptException:
		pass
