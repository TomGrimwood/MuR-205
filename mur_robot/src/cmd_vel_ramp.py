#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def computeRampProfile():
	rospy.init_node('cmd_vel_ramp_node', anonymous=True)
	print "Node is running..."

	
	rospy.set_param("break",1)

	ramp_cmd_wheel_velocities=[0,0]

	stepsize=0.05

	frequency=20 #hertz
   

	rate = rospy.Rate(frequency*10) 

	rate_inner=rospy.Rate(frequency)

	while not rospy.is_shutdown() and rospy.get_param("break")==1:
		
		cmd_wheel_velocities=rospy.get_param("mur/cmd_wheel_velocities")

	   
		if cmd_wheel_velocities[0] > ramp_cmd_wheel_velocities[0] or cmd_wheel_velocities[1] > ramp_cmd_wheel_velocities[1]:
			while  not rospy.is_shutdown():

				if cmd_wheel_velocities[0] > ramp_cmd_wheel_velocities[0]:
					ramp_cmd_wheel_velocities[0]=ramp_cmd_wheel_velocities[0]+stepsize
				else:
					pass

				if cmd_wheel_velocities[1] > ramp_cmd_wheel_velocities[1]:
					ramp_cmd_wheel_velocities[1]=ramp_cmd_wheel_velocities[1]+stepsize
				else:
					pass             
				
				print "1ramp_cmd_wheel_velocities: ",ramp_cmd_wheel_velocities
				rospy.set_param("mur/ramp_cmd_wheel_velocities",ramp_cmd_wheel_velocities)

				if cmd_wheel_velocities[0] < ramp_cmd_wheel_velocities[0] and cmd_wheel_velocities[1] < ramp_cmd_wheel_velocities[1]:
					break
				rate_inner.sleep()
		
		

		'''
		if cmd_wheel_velocities[0] < ramp_cmd_wheel_velocities[0] or cmd_wheel_velocities[1] < ramp_cmd_wheel_velocities[1]:
			while  not rospy.is_shutdown():

				if cmd_wheel_velocities[0] < ramp_cmd_wheel_velocities[0]:
					ramp_cmd_wheel_velocities[0]=ramp_cmd_wheel_velocities[0]-stepsize
				else:
					pass

				if cmd_wheel_velocities[1] < ramp_cmd_wheel_velocities[1]:
					ramp_cmd_wheel_velocities[1]=ramp_cmd_wheel_velocities[1]-stepsize
				else:
					pass             
				
				print "2ramp_cmd_wheel_velocities: ",ramp_cmd_wheel_velocities
				rospy.set_param("mur/ramp_cmd_wheel_velocities",ramp_cmd_wheel_velocities)

				if cmd_wheel_velocities[0] > ramp_cmd_wheel_velocities[0] and cmd_wheel_velocities[1] > ramp_cmd_wheel_velocities[1]:
					break
				rate_inner.sleep()
		'''
		
		
		
		
		
		
		rate.sleep()
	rospy.set_param("mur/ramp_cmd_wheel_velocities",[0,0])

if __name__ == '__main__':
	try:
		computeRampProfile()
	except rospy.ROSInterruptException:
		pass
