#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Remapper():

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	pub=rospy.Publisher('cmd_vel', Twist, queue_size=10)

	def callback(self,twist):
		#print data

		#self.pub.publish(twist)

	
		wheel_vel_left=(twist.linear.x-(self.wheel_separation/2)*twist.angular.z)/self.wheel_radius
		wheel_vel_right=(twist.linear.x+(self.wheel_separation/2)*twist.angular.z)/self.wheel_radius

		cartesian_velocities=[wheel_vel_left,wheel_vel_right]
		
		rospy.set_param("mur/cmd_wheel_velocities",cartesian_velocities)
		
		
	def listener(self):

		
		
		rospy.init_node('cmd_remapper', anonymous=True)

		rospy.Subscriber("/smooth_cmd_vel", Twist, self.callback)
		print("Remapping smooth_cmd_vel to params ...")

		
		rospy.spin()

if __name__ == '__main__':
	remap=Remapper()
	remap.listener()
	