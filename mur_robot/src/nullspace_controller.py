#!/usr/bin/env python
import rospy
import tf
import numpy as np
from std_msgs.msg import String
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from pose_subscriber import getPoseUR, getPoseMiR
from ur_msgs.msg import RobotModeDataMsg


def transformer(listener):

	# getting the Transformation matrix from UR_TCP to UR_Base
	
	listener.waitForTransform('/base','/tool0_controller',rospy.Time(0), rospy.Duration(3))
	(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	print "Got new tf"

	

	urbase_T_urtcp=tf.transformations.quaternion_matrix(rot_quat)
	urbase_T_urtcp[0][3]=trans[0]
	urbase_T_urtcp[1][3]=trans[1]
	urbase_T_urtcp[2][3]=trans[2]
	#print "\nurbase_T_urtcp: \n",urbase_T_urtcp
	

	
	# Definition of the rigid transformation between UR_base and MiR_base
	R_z_180 = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
	t_mir_ur = [0.250, -0.145, 0.445]
	mirbase_T_urbase=np.array([[R_z_180[0][0],R_z_180[0][1],R_z_180[0][2],t_mir_ur[0]],
						[R_z_180[1][0],R_z_180[1][1],R_z_180[1][2],t_mir_ur[1]],
						[R_z_180[2][0],R_z_180[2][1],R_z_180[2][2],t_mir_ur[2]],
						[0,0,0,1]])

	#print "\n\nmirbase_T_urbase: \n",mirbase_T_urbase
	

	
	# getting the transformation matrix from the mir_base
	mir_pose = rospy.wait_for_message('/robot_pose', Pose)
	mir_trans=[mir_pose.position.x,mir_pose.position.y,mir_pose.position.z]
	mir_rot_quat=[mir_pose.orientation.x,mir_pose.orientation.y,mir_pose.orientation.z,mir_pose.orientation.w]
	#print "mir_pose: ",mir_pose.orientation
	world_T_mirbase=tf.transformations.quaternion_matrix(mir_rot_quat)
	world_T_mirbase[0][3]=mir_trans[0]
	world_T_mirbase[1][3]=mir_trans[1]
	world_T_mirbase[2][3]=mir_trans[2]
	#print "\n\world_T_mirbase: \n",world_T_mirbase

	
	
	
	#T1=np.dot(world_T_mirbase,mirbase_T_urbase)
	mirbase_T_urtcp=np.dot(mirbase_T_urbase,urbase_T_urtcp)
	
	#point_urtcp=[0,0,0,1]
	#point_mirbase= np.dot(mirbase_T_urtcp, point_urtcp)
	#point_world= np.dot(world_T_mirbase,point_mirbase)

	world_T_urtcp=np.dot(world_T_mirbase,mirbase_T_urtcp)


	

	#print "point_mirbase: ",point_mirbase

	#print "point_world: ",point_world
	
	return world_T_urtcp

def controller():

	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

	var=raw_input("Go on:")

	pub.publish('movej('+str(rospy.get_param("arm/null_space_init_j"))+', a=1.2, v=0.25, t=0, r=0)')
	break_loop=True
	while(break_loop):
	  rospy.sleep(0.01)
	  msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
	  break_loop=msg.is_program_running

	listener = tf.TransformListener()
	target_Trafo= transformer(listener)


	P_x=2
	P_y=-2

	I_x=0
	I_y=0

	D_x=0
	D_y=0

	e_sum_x=0
	e_sum_y=0

	error_diff_x=0
	error_diff_y=0

	prev_error_x=0
	prev_error_y=0

	var=raw_input("Go on:")

	
	rospy.set_param("break",1)
	
	
	while rospy.get_param('break')==1:
		rospy.sleep(0.05)
		try:
			current_point_world=transformer(listener)
		except:
			#print (e)
			pass
		R_current=current_point_world[:-1,:-1]

		

	
		error_x =target_Trafo[0][3] - current_point_world[0][3]
		error_y =target_Trafo[1][3] - current_point_world[1][3]

		error_tcp=np.array([error_x,error_y,0])

		rotated_errors=np.dot(R_current,error_tcp)

		e_sum_x=e_sum_x +rotated_errors[0]
		e_sum_y=e_sum_y +rotated_errors[1]

		error_diff_x=rotated_errors[0]-prev_error_x
		error_diff_y=rotated_errors[1]-prev_error_y

		speedx= P_x * rotated_errors[0] + I_x*e_sum_x + D_x*error_diff_x
		speedy= P_y * rotated_errors[1] + I_y*e_sum_y + D_y*error_diff_y

		print "\n\nspeedx: ",speedx, "speedy: ",speedy

		pub.publish('speedl(['+str(speedx)+','+str(speedy)+',0,0,0,0], 0.5, 0.5)')

		prev_error_x=rotated_errors[0]
		prev_error_y=rotated_errors[1]
	
	pub.publish('Stopj(2)')




if __name__ == '__main__':
	rospy.init_node('pose_transformer', anonymous=True)

	#target_Trafo= transformer()

	controller()

	