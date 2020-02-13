#!/usr/bin/env python
import rospy
import time
import tf
import numpy as np
import math
from std_msgs.msg import String
from std_msgs.msg import Float32
#from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from pose_subscriber import getPoseUR, getPoseMiR
from ur_msgs.msg import RobotModeDataMsg
from sensor_msgs.msg import JointState
#def callback(data):
	
   

	#laeuft fluessig
	#listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(0.1))
	#(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	#print trans

def transformer(mir_pose):

	# getting the Transformation matrix from UR_TCP to UR_Base
	global listener
	#listener = tf.TransformListener()
	listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))
	(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	

	urbase_T_urtcp=tf.transformations.quaternion_matrix(rot_quat)
	urbase_T_urtcp[0][3]=trans[0]
	urbase_T_urtcp[1][3]=trans[1]
	urbase_T_urtcp[2][3]=trans[2]
	#print "\nurbase_T_urtcp: \n",urbase_T_urtcp
	

	
	# Definition of the rigid transformation between UR_base and MiR_base
	'''
	R_z_180 = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]
	t_mir_ur = [0.250, -0.145, 0.445]
	
	mirbase_T_urbase=np.array([[R_z_180[0][0],R_z_180[0][1],R_z_180[0][2],t_mir_ur[0]],
						[R_z_180[1][0],R_z_180[1][1],R_z_180[1][2],t_mir_ur[1]],
						[R_z_180[2][0],R_z_180[2][1],R_z_180[2][2],t_mir_ur[2]],
						[0,0,0,1]])
	'''

	#calibrated values
	t_mir_ur = [0.244311483849, -0.140242770172, 0.450477350561]
	
	mirbase_T_urbase=tf.transformations.euler_matrix(0,0,math.pi - 0.0154892745077)
	mirbase_T_urbase[0][3]=t_mir_ur[0]
	mirbase_T_urbase[1][3]=t_mir_ur[1]
	mirbase_T_urbase[2][3]=t_mir_ur[2]



	print "\n\nmirbase_T_urbase: \n",mirbase_T_urbase
	

	
	# getting the transformation matrix from the mir_base
	#mir_pose = rospy.wait_for_message('/robot_pose', Pose)
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

	world_T_urbase=np.dot(world_T_mirbase,mirbase_T_urbase)
	urbase_R_world=np.linalg.inv(world_T_urbase[:-1,:-1])
	
	#point_urtcp=[0,0,0,1]
	#point_mirbase= np.dot(mirbase_T_urtcp, point_urtcp)
	#point_world= np.dot(world_T_mirbase,point_mirbase)

	world_T_urtcp=np.dot(world_T_mirbase,mirbase_T_urtcp)


	

	#print "point_mirbase: ",point_mirbase

	#print "point_world: ",point_world
	
	return world_T_urtcp, urbase_R_world

def controller(data):
	current_time=rospy.rostime.get_time()
	current_timeros=rospy.get_rostime()
	global pub
	global rqt_pub_x
	global rqt_pub_y
	global rqt_pub_speed_x
	global rqt_pub_speed_y
	global target_Trafo
	global P_x
	
	global P_y
	

	global I_x
	
	global I_y
	

	global D_x
	
	global D_y
	

	#global e_sum_x
	global e_sum_x_array
	global e_sum_y_array
	global e_sum_buffer_length
	#global e_sum_y
	

	global error_diff_x
	
	global error_diff_y

	global D_time_diff_counter
	

	global prev_error_x

	global prev_error_y

	global target_alpha

	global P_a



	global last_publ_time

	

	diff_time=current_timeros-last_publ_time
	#print "current_time: ",current_time, "diff_time: ", diff_time,"rostime: ",current_timeros
	
	

	if diff_time.to_nsec()>5000000:

	


		

		current_point_world,urbase_R_world=transformer(data)
		R_current=current_point_world[:-1,:-1]

		alpha,beta,gamma=tf.transformations.euler_from_matrix(R_current,axes='sxyz')

		#print alpha, beta, gamma 

		


		error_x =target_Trafo[0][3] - current_point_world[0][3]
		error_y =target_Trafo[1][3] - current_point_world[1][3]
		error_g =target_gamma - gamma

		error_tcp=np.array([error_x,error_y,0])

		rotated_errors=np.dot(urbase_R_world,error_tcp)

		e_sum_x_array.insert(0,error_x)
		e_sum_y_array.insert(0,error_y)

		if len(e_sum_x_array)>e_sum_buffer_length:
			e_sum_x_array.pop(e_sum_buffer_length)
		if len(e_sum_y_array)>e_sum_buffer_length:
			e_sum_y_array.pop(e_sum_buffer_length)

		#print e_sum_x_array
		#print len(e_sum_x_array)

		e_sum_x=sum(e_sum_x_array)
		e_sum_y=sum(e_sum_y_array)

		#print e_sum_x
		


		#old
		#e_sum_x=e_sum_x +rotated_errors[0]
		#e_sum_y=e_sum_y +rotated_errors[1]

		#UPDATING THE DIFF_ERROR STACK
		prev_error_x.insert(0,rotated_errors[0])
		prev_error_y.insert(0,rotated_errors[1])
		
		if len(prev_error_x)>e_diff_buffer_size:
			prev_error_x.pop(e_diff_buffer_size)
		if len(prev_error_y)>e_diff_buffer_size:
			prev_error_y.pop(e_diff_buffer_size)

		#print prev_error_x
		#print prev_error_y



		error_diff_x=-(rotated_errors[0]-prev_error_x[len(prev_error_x)-1])
		error_diff_y=-(rotated_errors[1]-prev_error_y[len(prev_error_y)-1])

		#print rotated_errors[0], " - ",prev_error_x[len(prev_error_x)-1]



		e_sum_x_threshold=10
		# LIMIT e_sum_x:
		if e_sum_x > e_sum_x_threshold:
			e_sum_x =e_sum_x_threshold
		elif e_sum_x < -e_sum_x_threshold:
			e_sum_x =-e_sum_x_threshold
		
		e_sum_y_threshold=10
		# LIMIT e_sum_x:
		if e_sum_y > e_sum_y_threshold:
			e_sum_y =e_sum_y_threshold
		elif e_sum_y < -e_sum_y_threshold:
			e_sum_y =-e_sum_y_threshold

		error_diff_x_threshold=10
		# LIMIT error_diff_x:
		if error_diff_x > error_diff_x_threshold:
			error_diff_x =error_diff_x_threshold
		elif error_diff_x < -error_diff_x_threshold:
			error_diff_x =-error_diff_x_threshold

		error_diff_y_threshold=10
		# LIMIT error_diff_y:
		if error_diff_y > error_diff_y_threshold:
			error_diff_y =error_diff_y_threshold
		elif error_diff_y < -error_diff_y_threshold:
			error_diff_y =-error_diff_y_threshold





		# CONTROLLER EQUATION
		speedx= P_x * rotated_errors[0] + I_x*e_sum_x + D_x*error_diff_x
		speedy= P_y * rotated_errors[1] + I_y*e_sum_y + D_y*error_diff_y
		speedg= P_g * error_g



		speedx_threshold=1
		speedy_threshold=1

		# LIMIT speedx:
		if speedx > speedx_threshold:
			speedx =speedx_threshold
		elif speedx < -speedx_threshold:
			speedx =-speedx_threshold

		# LIMIT speedy:
		if speedy > speedy_threshold:
			speedy =speedy_threshold
		elif speedy < -speedy_threshold:
			speedy =-speedy_threshold

		




		
		print round(rotated_errors[0],4),round(rotated_errors[1],4)
		print round(e_sum_x,4),round(e_sum_y,4)
		print round(error_diff_x,4),round(error_diff_y,4)
		print "speedx: ",round(speedx,4), "speedy: ",round(speedy,4),"\n\n"
	
		#print "pub: ",diff_time
		#last_publ_time=rospy.rostime.get_time()
		
		pub.publish('speedl(['+str(speedx)+','+str(speedy)+',0,0,0,'+str(speedg)+'], 0.8, 0.8)')


		pub_time=rospy.get_rostime()
		rqt_pub_x.publish(rotated_errors[0])
		rqt_pub_y.publish(rotated_errors[1])
		rqt_pub_speed_x.publish(speedx)
		rqt_pub_speed_y.publish(speedy)
		
		last_publ_time=rospy.get_rostime()
		#print last_publ_time-pub_time


		
		#prev_error_x=rotated_errors[0]
		#prev_error_y=rotated_errors[1]
	
	
	
def getJointPoseUR():


	var=raw_input("Go on1:")

	global pub
	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

	global rqt_pub_x
	rqt_pub_x = rospy.Publisher('/mur/error/y', Float32, queue_size=1)
	global rqt_pub_y
	rqt_pub_y = rospy.Publisher('/mur/error/x', Float32, queue_size=1)
	global rqt_pub_speed_x
	rqt_pub_speed_x = rospy.Publisher('/mur/speed/x', Float32, queue_size=1)
	global rqt_pub_speed_y
	rqt_pub_speed_y = rospy.Publisher('/mur/speed/y', Float32, queue_size=1)
	rospy.sleep(0.5)
   
	pub.publish('movej('+str(rospy.get_param("arm/null_space_init_j"))+', a=1.2, v=0.5, t=0, r=0)')
	
	break_loop=True
	
	while(break_loop):
	  rospy.sleep(0.01)
	  msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
	  break_loop=msg.is_program_running


	mir_pose = rospy.wait_for_message('/robot_pose', Pose)
	mir_trans=[mir_pose.position.x,mir_pose.position.y,mir_pose.position.z]
	global listener
	listener = tf.TransformListener()
	
	global target_Trafo
	target_Trafo,urbase_R_world= transformer(mir_pose)

	target_angles=tf.transformations.euler_from_matrix(target_Trafo[:-1,:-1],axes='sxyz')

	global target_alpha
	target_alpha = target_angles[0]
	print "target_alpha: ",target_alpha

	global target_beta
	target_beta = target_angles[1]
	print "target_beta: ",target_beta

	global target_gamma
	target_gamma = target_angles[2]
	print "target_gamma: ",target_gamma




	global P_x
	P_x=3
	global P_y
	P_y=3

	global I_x
	I_x=0
	global I_y
	I_y=0

	global D_x
	D_x=0
	global D_y
	D_y=0



	global e_sum_x_array
	e_sum_x_array=[]
	#e_sum_x=0
	global e_sum_y_array
	e_sum_y_array=[]
	global e_sum_buffer_length
	e_sum_buffer_length=10


	global error_diff_x
	error_diff_x=0
	global error_diff_y
	error_diff_y=0

	

	global prev_error_x
	prev_error_x=[]
	prev_error_x.append(0)
	global prev_error_y
	prev_error_y=[]
	prev_error_y.append(0)

	global e_diff_buffer_size
	e_diff_buffer_size=2


	global P_g
	P_g=3


	global last_publ_time
	last_publ_time=rospy.get_rostime()
	rospy.sleep(0.5)

	var=raw_input("Go on:")








	
	
	
	rospy.Subscriber("robot_pose", Pose, controller,queue_size=1)
	rospy.spin()

	pub.publish('Stopj(2)')


	

	

	
if __name__ == '__main__':
	rospy.init_node('JointPose_subscriber', anonymous=True)

	'''
	rospy.set_param("break",1)
	while rospy.get_param("break")==1:

		print time.time()
		rospy.sleep(0.01)
	'''

	getJointPoseUR()