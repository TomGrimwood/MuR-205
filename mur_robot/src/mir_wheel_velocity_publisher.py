#!/usr/bin/env python

#Author: Florian Heilemann

import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from mur_robot.msg import wheel_vel

class wheel_velocity():

	wheel_vel_pub = rospy.Publisher('/mir/wheel_velocity', wheel_vel, queue_size=1)
	rospy.sleep(0.2)

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	def callback(self,data):

		x_lin_vel=data.twist.twist.linear.x
		z_rot_vel=data.twist.twist.angular.z

		wheel_velocities=wheel_vel()

		wheel_vel_left=(x_lin_vel-(self.wheel_separation/2)*z_rot_vel)/self.wheel_radius
		wheel_vel_right=(x_lin_vel+(self.wheel_separation/2)*z_rot_vel)/self.wheel_radius

		wheel_velocities.left_vel=wheel_vel_left
		wheel_velocities.right_vel=wheel_vel_right

		wheel_velocities.header.stamp=rospy.Time.now()

		self.wheel_vel_pub.publish(wheel_velocities)
		
	def odomListener(self):

		rospy.Subscriber("/odom",Odometry, self.callback)
		rospy.spin()

if __name__ == '__main__':

	rospy.init_node('mir_wheel_velocity_publisher_node', anonymous=True)
	
	mir_wheel_velocity=wheel_velocity()
	rospy.sleep(0.2)
	print ("initialized publishers")
	mir_wheel_velocity.odomListener()
