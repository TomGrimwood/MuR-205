#!/usr/bin/env python
import rospy
import sys
import time
import tf
import numpy as np
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from std_msgs.msg import Float32
from moveit_commander.conversions import pose_to_list
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import message_filters
from mur_robot.msg import wheel_vel


class JacobianPublisher():
	rospy.init_node('jacobian_publisher_node', anonymous=True)

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	listener = tf.TransformListener()

	old_time=rospy.Time.now()

	t_mir_ur = [0.244311483849, -0.140242770172, 0.450477350561]
		
	mirbase_T_urbase=tf.transformations.euler_matrix(0,0,( 0.0154892745077))# Since get_jacobian Frame is rotated about pi, T looks like this
	mirbase_T_urbase[0][3]=t_mir_ur[0]
	mirbase_T_urbase[1][3]=t_mir_ur[1]
	mirbase_T_urbase[2][3]=t_mir_ur[2]

	mirbase_R_urbase=mirbase_T_urbase[:-1,0:-1]

	jacobian_pub = rospy.Publisher('/mur/jacobian', String, queue_size=10)


	
	

	try:
		print "start"
		group = moveit_commander.MoveGroupCommander('manipulator')
	except Exception as e: 
		print(e)
	joint_values=np.array([])




	def messageFilter(self):

		joint_states_sub = message_filters.Subscriber('/joint_states', JointState)
		odom_sub = message_filters.Subscriber('/odom', Odometry)
		wheel_vel_sub = message_filters.Subscriber('/mir/wheel_velocity', wheel_vel)
		

		ts = message_filters.ApproximateTimeSynchronizer([joint_states_sub,odom_sub,wheel_vel_sub], 10,55)
		
		ts.registerCallback(self.callback)
		rospy.spin()

	def callback(self,joint_states,odom,wheel_vel):
		

		


		self.listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))

		(trans,rot_quat) = self.listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

		

		joint_states_array=[joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
		J_ur=self.group.get_jacobian_matrix(joint_states_array)

		# MiR Jacobian
		#mir_pose = rospy.wait_for_message('/odom', Odometry)
		mir_quaterion=[odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w]
		orientation_euler=tf.transformations.euler_from_quaternion(mir_quaterion)
		current_theta=orientation_euler[2]

		
		q_dot_left=wheel_vel.left_vel
		q_dot_right=wheel_vel.right_vel

		J_mir=np.array([[math.cos(current_theta)*(self.wheel_radius/2),math.cos(current_theta)*(self.wheel_radius/2)],
				[math.sin(current_theta)*(self.wheel_radius/2),math.sin(current_theta)*(self.wheel_radius/2)],
				[0,0],
				[0,0],
				[0,0],
				[- self.wheel_radius/self.wheel_separation,self.wheel_radius/self.wheel_separation]])

		

		
		#q_dot=[joint_states.velocity[0],joint_states.velocity[1],joint_states.velocity[2],joint_states.velocity[3],joint_states.velocity[4],joint_states.velocity[5],q_dot_left,q_dot_right]
		#q_dot_ur=q_dot[:-2]

		#X_dot_ur=np.dot(J_ur,q_dot_ur)

		

		urbase_T_urtcp=tf.transformations.quaternion_matrix(rot_quat)
		urbase_T_urtcp[0][3]=trans[0]
		urbase_T_urtcp[1][3]=trans[1]
		urbase_T_urtcp[2][3]=trans[2]

		trans.append(0)
		
		
		#skew vector
		vec_mir_base_ur_tcp=np.dot(self.mirbase_T_urbase,trans)
		

		# Computing the skew  matrix :
		# Translational Speed of EE is depending on the current Distance from mir_base <--> EE and the rotational speed of the mir (Cross-Product)
		skew_matrix_small=np.array([
							[0,vec_mir_base_ur_tcp[2],-vec_mir_base_ur_tcp[1]],
							[-vec_mir_base_ur_tcp[2],0,vec_mir_base_ur_tcp[0]],
							[vec_mir_base_ur_tcp[1],-vec_mir_base_ur_tcp[0],0]
							])

		
		# Gardner & Velinsky [Eq. 3]
		eye=np.identity(3)
		zeros_3_3=np.zeros((3,3))
		skew_matrix1=np.concatenate((eye,skew_matrix_small),1)
		skew_matrix2=np.concatenate((zeros_3_3,eye),1)
		skew_matrix=np.concatenate((skew_matrix1,skew_matrix2),0)

		J_mir_skewed=np.dot(skew_matrix,J_mir)


		
		

		
		# Transform the orientation from the ur_base frame to the mir_base frame. Only Rotation is important, since we are only regarding velocities.
		mir_R_ur_1=np.concatenate((self.mirbase_R_urbase,zeros_3_3),1)
		mir_R_ur_2=np.concatenate((zeros_3_3,self.mirbase_R_urbase),1)
		mir_R_ur=np.concatenate((mir_R_ur_1,mir_R_ur_2),0)
		
		mir_J_ur_rigid_rotation=np.dot(mir_R_ur,J_ur)

		#print "mir_J_ur_rigid_rotation: \n",mir_J_ur_rigid_rotation


		# Transforming the Jacobian into the world frame using the current planar orientation theta 
		world_R_mir_3x3=tf.transformations.euler_matrix(orientation_euler[0],orientation_euler[1],orientation_euler[2])
		world_R_mir_3x3=world_R_mir_3x3[:-1,0:-1]

		world_R_mir_1=np.concatenate((world_R_mir_3x3,zeros_3_3),1)
		world_R_mir_2=np.concatenate((zeros_3_3,world_R_mir_3x3),1)
		world_R_mir=np.concatenate((world_R_mir_1,world_R_mir_2),0)

		world_J_ur=np.dot(world_R_mir,mir_J_ur_rigid_rotation)
		

		J=np.concatenate((world_J_ur,J_mir_skewed),1)

		#print "\nJ: \n",J

		#X_dot=np.dot(J,q_dot)

		#print "X_dot: ",X_dot

		hello_str="[1,0,0,0]"

		self.jacobian_pub.publish(hello_str)

		

		
		
		

if __name__ == '__main__':
	try:
		
		J_pub=JacobianPublisher()
		J_pub.messageFilter()
		
	except rospy.ROSInterruptException:
		pass