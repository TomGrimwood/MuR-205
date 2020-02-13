#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist

'''
def callback(data):
	print "\n\nX: ",data.wrench.force.x
	print "Y: ",data.wrench.force.y

	velocity_command=Twist()
	velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=4)

	x_vel_threshold=1

	velocity_command.linear.x = data.wrench.force.x * -0.03
	velocity_command.angular.z = data.wrench.force.y * -0.03

	if abs(velocity_command.linear.x) < 0.1:
		velocity_command.linear.x =0
	if velocity_command.linear.x > x_vel_threshold:
		velocity_command.linear.x =x_vel_threshold
	elif velocity_command.linear.x<-x_vel_threshold:
		velocity_command.linear.x=-x_vel_threshold

	if abs(velocity_command.angular.z) < 0.1:
		velocity_command.angular.z =0
	if velocity_command.angular.z > 0.5:
		velocity_command.angular.z =0.5
	elif velocity_command.angular.z<-0.5:
		velocity_command.angular.z=-0.5

	print "\n\nX_vel: ",velocity_command.linear.x 
	print "Z_rot_vel: ",velocity_command.angular.z,"\n\n\n\n"



	velocity_publisher.publish(velocity_command)
'''


	
def commandTrajectory():

	
	

	velocity_command=Twist()
	velocity_publisher = rospy.Publisher('raw_cmd_vel', Twist, queue_size=4)
	rospy.set_param("mur/cmd_wheel_velocities",[0,0])
	rospy.sleep(1)
	var1=raw_input("Current Trajectory: Load")

	velocity_command=Twist()
	velocity_command.linear.x = 0
	velocity_command.linear.y = 0
	velocity_command.linear.z = 0
	velocity_command.angular.x = 0
	velocity_command.angular.y = 0
	velocity_command.angular.z = 0

	#move(velocity_publisher,0,0.15,10)
	#
	#move(velocity_publisher,0.02607883272953,0,10)
	#rospy.set_param("mur/cmd_wheel_velocities",[0,0])


	'''
	#Nullspace
	
	move(velocity_publisher,0.0607883272953,-0.121576654591,8)
	move(velocity_publisher,0,0,2)

	move(velocity_publisher,0,-0.121576654591,3)
	move(velocity_publisher,0,0,2)

	move(velocity_publisher,0.0607883272953,0,3)
	move(velocity_publisher,0,0,2)

	move(velocity_publisher,0,0.121576654591,11)
	move(velocity_publisher,0,0,2)

	move(velocity_publisher,-0.0607883272953,0,5)
	move(velocity_publisher,0,0,2)
	'''

	
	#Nullspace smooth
	move(velocity_publisher,0.0607883272953,-0.121576654591,12)
	move(velocity_publisher,0,0,2)

	#move(velocity_publisher,0,-0.121576654591,5)
	move(velocity_publisher,0,0,2)

	move(velocity_publisher,0.0607883272953,0,4)
	move(velocity_publisher,0,0,4)

	move(velocity_publisher,0,0.121576654591,12)
	move(velocity_publisher,0,0,4)
	
	move(velocity_publisher,-0.0607883272953,0,5)
	move(velocity_publisher,0,0,4)
	'''




	
	# spachula
	
	pause=3.5
	rospy.sleep(pause)
	move(velocity_publisher,0.0607883272953,0,5)
	print"1"
	move(velocity_publisher,0,0,1)

	
	move(velocity_publisher,0.0607883272953,0.121576654591,4)
	print"2"
	#move(velocity_publisher,0,0,2)

	
	move(velocity_publisher,0.0607883272953,-0.121576654591,8)
	print"3"
	move(velocity_publisher,0,0,2)

	
	move(velocity_publisher,0.0607883272953,0.121576654591,4)
	print"4"
	move(velocity_publisher,0,0,2)

	
	move(velocity_publisher,0.0607883272953,0,5)
	print"5"
	move(velocity_publisher,0,0,2)
	'''
	rospy.set_param("break",0)
	

def move(velocity_publisher,x,theta,duration):

	velocity_command=Twist()
	velocity_command.linear.x = 0
	velocity_command.linear.y = 0
	velocity_command.linear.z = 0
	velocity_command.angular.x = 0
	velocity_command.angular.y = 0
	velocity_command.angular.z = 0
	velocity_command.linear.x = x
	velocity_command.angular.z = theta
	

	print "\n\nX_vel: ",velocity_command.linear.x 
	print "Z_rot_vel: ",velocity_command.angular.z,"\n\n\n\n"

	cmd_wheel_velocities=computeWheelVelocities(velocity_command.linear.x,velocity_command.angular.z)

	for i in range(duration*10):
		velocity_publisher.publish(velocity_command)
		#rospy.set_param("mur/cmd_wheel_velocities",cmd_wheel_velocities)
		print i,cmd_wheel_velocities
		rospy.sleep(0.1)

	

def computeWheelVelocities(x_lin_vel,z_rot_vel):

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	wheel_vel_left=(x_lin_vel-(wheel_separation/2)*z_rot_vel)/wheel_radius
	wheel_vel_right=(x_lin_vel+(wheel_separation/2)*z_rot_vel)/wheel_radius

	cmd_wheel_velocities=[wheel_vel_left,wheel_vel_right]
	return cmd_wheel_velocities





	
if __name__ == '__main__':
	rospy.init_node('guide_robot_node', anonymous=True)
	#var=raw_input("Go: ")
	commandTrajectory()
