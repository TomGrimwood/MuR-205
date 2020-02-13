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


class JacobianPublisher():

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	
	

	try:
		print "start"
		group = moveit_commander.MoveGroupCommander('manipulator')
	except Exception as e: 
		print(e)
	joint_values=np.array([])



	def publish(self):
		pub = rospy.Publisher('/mur/jacobian', String, queue_size=10)
		listener = tf.TransformListener()

		#mirbase_R_urbase=tf.transformations.euler_matrix(0,0,-math.pi + 0.0154892745077)
		#mirbase_R_urbase=mirbase_R_urbase[:-1,0:-1] #cut off last row and line in order to shape a 3x3 matrix

		# Transformation between mir_base and ur_base - with calibrated values
		t_mir_ur = [0.244311483849, -0.140242770172, 0.450477350561]
		
		mirbase_T_urbase=tf.transformations.euler_matrix(0,0,( 0.0154892745077))# Since get_jacobian Frame is rotated about pi, T looks like this
		mirbase_T_urbase[0][3]=t_mir_ur[0]
		mirbase_T_urbase[1][3]=t_mir_ur[1]
		mirbase_T_urbase[2][3]=t_mir_ur[2]

		mirbase_R_urbase=mirbase_T_urbase[:-1,0:-1]
		
		
		rate = rospy.Rate(100) # x hz
		while not rospy.is_shutdown():
			
			# UR-Jacobian
			try:
				joint_states=rospy.wait_for_message('/joint_states', JointState)
			except Exception as e:
				print(e)
			joint_values= joint_states.position
			joint_states_array=[joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]]
			J_ur=self.group.get_jacobian_matrix(joint_states_array)
						
			
			#print "J_ur: \n",J_ur




			# MiR Jacobian
			mir_pose = rospy.wait_for_message('/odom', Odometry)
			mir_quaterion=[mir_pose.pose.pose.orientation.x,mir_pose.pose.pose.orientation.y,mir_pose.pose.pose.orientation.z,mir_pose.pose.pose.orientation.w]
			orientation_euler=tf.transformations.euler_from_quaternion(mir_quaterion)
			current_theta=orientation_euler[2]

			try:
				q_dot_left_msg=rospy.wait_for_message('/mir/wheel_velocity/left',Float32)
				q_dot_right_msg=rospy.wait_for_message('/mir/wheel_velocity/right',Float32)
			except Exception as e:
				print(e)
			q_dot_left=q_dot_left_msg.data
			q_dot_right=q_dot_right_msg.data

			J_mir=np.array([[math.cos(current_theta)*(self.wheel_radius/2),math.cos(current_theta)*(self.wheel_radius/2)],
					[math.sin(current_theta)*(self.wheel_radius/2),math.sin(current_theta)*(self.wheel_radius/2)],
					[0,0],
					[0,0],
					[0,0],
					[- self.wheel_radius/self.wheel_separation,self.wheel_radius/self.wheel_separation]])

			

			
			q_dot=[joint_states.velocity[0],joint_states.velocity[1],joint_states.velocity[2],joint_states.velocity[3],joint_states.velocity[4],joint_states.velocity[5],q_dot_left,q_dot_right]
			q_dot_ur=q_dot[:-2]

			X_dot_ur=np.dot(J_ur,q_dot_ur)
			#print "X_dot_ur: \n",X_dot_ur

			# getting the Transformation matrix from UR_TCP to UR_Base needed for skew matrix
			listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))
			(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
			

			urbase_T_urtcp=tf.transformations.quaternion_matrix(rot_quat)
			urbase_T_urtcp[0][3]=trans[0]
			urbase_T_urtcp[1][3]=trans[1]
			urbase_T_urtcp[2][3]=trans[2]

			trans.append(0)
			
			
			#skew vector
			vec_mir_base_ur_tcp=np.dot(mirbase_T_urbase,trans)
			

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
			mir_R_ur_1=np.concatenate((mirbase_R_urbase,zeros_3_3),1)
			mir_R_ur_2=np.concatenate((zeros_3_3,mirbase_R_urbase),1)
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

			X_dot=np.dot(J,q_dot)

			#print "X_dot: ",X_dot













			
			hello_str="[1,0,0,0]"

			pub.publish(hello_str)
			rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('jacobian_publisher_node', anonymous=True)
		J_pub=JacobianPublisher()
		J_pub.publish()
		
	except rospy.ROSInterruptException:
		pass