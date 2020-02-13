#!/usr/bin/env python
import sys
import rospy
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
#from std_msgs.msg import String
#from std_msgs.msg import Float32
#from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Pose
#from nav_msgs.msg import Odometry
#from pose_subscriber import getPoseUR, getPoseMiR
#from ur_msgs.msg import RobotModeDataMsg
#from sensor_msgs.msg import JointState


def differentialKinematics():


	#wheel_radius=0.05 #meters
	#wheel_separation=0.5

	wheel_separation = 0.445208
	wheel_radius = 0.0625


	mir_pose = rospy.wait_for_message('/odom', Odometry)
	mir_quaterion=[mir_pose.orientation.x,mir_pose.orientation.y,mir_pose.orientation.z,mir_pose.orientation.w]
	orientation_euler=tf.transformations.euler_from_quaternion(mir_quaterion)

	current_theta=orientation_euler[2]

	q_dot_left=rospy.wait_for_message('/mir/wheel_velocities/left',Float32)
	q_dot_right=rospy.wait_for_message('/mir/wheel_velocities/right',Float32)

	# raw Jacobian of MIR Platform
	J_mir=np.array([[math.cos(current_theta)*(wheel_radius/2),math.cos(current_theta)*(wheel_radius/2)],
					[math.sin(current_theta)*(wheel_radius/2),math.sin(current_theta)*(wheel_radius/2)],
					[- wheel_radius/wheel_separation,wheel_radius/wheel_separation]])

	q_dot_mir=[q_dot_left,q_dot_right]

	mir_vel_cartesian=np.dot(J_mir,q_dot_mir)

	print "mir_vel_cartesian: ",mir_vel_cartesian


	mir_R_ur=tf.transformations.euler_matrix(0,0,math.pi - 0.0154892745077)

	print "mir_R_ur: \n",mir_R_ur

	
	
	'''
	# getting the Transformation matrix from UR_TCP to UR_Base
	listener = tf.TransformListener()
	listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(0.05))
	(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	'''
	# Transformation between mir_base and ur_base - with calibrated values
	t_mir_ur = [0.244311483849, -0.140242770172, 0.450477350561]
	
	mirbase_T_urbase=tf.transformations.euler_matrix(0,0,math.pi - 0.0154892745077)
	mirbase_T_urbase[0][3]=t_mir_ur[0]
	mirbase_T_urbase[1][3]=t_mir_ur[1]
	mirbase_T_urbase[2][3]=t_mir_ur[2]

	vec_mir_base_ur_tcp=[1,1,1]#np.dot(mirbase_T_urbase,trans)

	skew_matrix=np.array([
						[0,vec_mir_base_ur_tcp[2],-vec_mir_base_ur_tcp[1]],
						[-vec_mir_base_ur_tcp[2],0,vec_mir_base_ur_tcp[0]],
						[vec_mir_base_ur_tcp[1],-vec_mir_base_ur_tcp[0],0]
						])

	
	robot = moveit_commander.RobotCommander()

	scene = moveit_commander.PlanningSceneInterface()

	try:
		print "start"
		group = moveit_commander.MoveGroupCommander('manipulator')
	except Exception as e: 
		print(e)
	joint_values=np.array([])
	print type(joint_values)
	joint_states=rospy.wait_for_message('/joint_states', JointState)
	joint_values= joint_states.position
	print type(joint_values)
	joint_states_array=[joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]]
	print type(joint_states_array)

	rospy.set_param('break',0)
	while rospy.get_param('break')==0:
		J=group.get_jacobian_matrix(joint_states_array)
		print "Jacobian: ",J
	

	print "end"







if __name__ == '__main__':
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('differential_kinematics_node', anonymous=True)

	differentialKinematics()

	