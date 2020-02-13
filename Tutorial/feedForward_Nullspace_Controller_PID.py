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
from geometry_msgs.msg import Twist
from ur_msgs.msg import RobotModeDataMsg


class JacobianPublisher():
	rospy.init_node('jacobian_publisher_node', anonymous=True)

	rospy.set_param("mur/cartesian_velocities",[0,0,0,0,0,0])
	rospy.set_param("mur/ramp_cmd_wheel_velocities",[0,0])

	rospy.set_param("mur/target_delta_xyz",[0,0,0,0,0,0])
	target_delta_xyz_old=rospy.get_param("mur/target_delta_xyz")
	

	wheel_separation = 0.445208
	wheel_radius = 0.0625

	robot = moveit_commander.RobotCommander()
	scene = moveit_commander.PlanningSceneInterface()

	listener = tf.TransformListener()




	start_time_pathplanning=time.time()

	t_mir_ur = [0.244311483849, -0.140242770172, 0.450477350561]
		
	mirbase_T_urbase=tf.transformations.euler_matrix(0,0,(0.0154892745077))# Since get_jacobian Frame is rotated about pi, T looks like this
	mirbase_T_urbase[0][3]=t_mir_ur[0]
	mirbase_T_urbase[1][3]=t_mir_ur[1]
	mirbase_T_urbase[2][3]=t_mir_ur[2]
	# Since dkp kinematics and moveit coordinate frames are rotated about pi, this defines the other transformation, which is needed for the skew_matrix, as seen below
	mirbase_T_urbase_dkp=tf.transformations.euler_matrix(0,0,(math.pi - 0.0154892745077))# Since get_jacobian Frame is rotated about pi, T looks like this
	mirbase_T_urbase_dkp[0][3]=t_mir_ur[0]
	mirbase_T_urbase_dkp[1][3]=t_mir_ur[1]
	mirbase_T_urbase_dkp[2][3]=t_mir_ur[2]

	mirbase_R_urbase=mirbase_T_urbase[:-1,0:-1]

	R_z_180=tf.transformations.euler_matrix(0,0,math.pi)

	jacobian_pub = rospy.Publisher('/mur/jacobian', String, queue_size=1)

	robot = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)

	cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	rospy.sleep(0.5)

	cmd_vel=Twist()
	cmd_vel.linear.x = 0
	cmd_vel.linear.y = 0
	cmd_vel.linear.z = 0
	cmd_vel.angular.x = 0
	cmd_vel.angular.y = 0
	cmd_vel.angular.z = 0


	J_ur=None

	####

	# initialize target_position
	listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))
	(trans_init,rot_quat_init) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	
	urbase_T_urtcp_init=tf.transformations.quaternion_matrix(rot_quat_init)
	urbase_T_urtcp_init[0][3]=trans_init[0]
	urbase_T_urtcp_init[1][3]=trans_init[1]
	urbase_T_urtcp_init[2][3]=trans_init[2]

	mir_pose_init = rospy.wait_for_message('/odom', Odometry)

	#end
	##





	# Controller Stuff
	#P:
	K_P=3
	K_P_O=3

	# I:
	K_I=0.1
	error_sum_x_array=[]
	error_sum_y_array=[]
	error_sum_z_array=[]
	e_sum_buffer_length=20

	# D:
	K_D=10
	prev_error_x=[]
	prev_error_y=[]
	prev_error_z=[]
	e_diff_buffer_size=3
	



	speed_threshold=1
	speedO_threshold=1
	####


	
	

	try:
		print "start"
		group = moveit_commander.MoveGroupCommander('manipulator')
	except Exception as e: 
		print(e)
	joint_values=np.array([])



	def extendJacobian(self,J):


		constraint_1=np.array([[0,0,0,0,0,0,1,0]])
		constraint_2=np.array([[0,0,0,0,0,0,0,1]])

		#print "J: \n",J
		#add constraints to jacobian
		J_extended=np.concatenate((J,constraint_1),0)
		J_extended=np.concatenate((J_extended,constraint_2),0)

		#print "J_extended: \n",J_extended

		rank_J_extended = np.linalg.matrix_rank(J_extended)
		#print "Rank of J_extended: ",rank_J_extended





		return J_extended






	def callback(self,joint_states,odom,wheel_vel):
		
		##############################
		
		#COMPUTATION OF MUR JACOBIAN :
		


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

		trans.append(1)

		#print "trans: ",trans

		#print "mirbase_T_urbase: \n",self.mirbase_T_urbase
		
		
		#skew vector
		vec_mir_base_ur_tcp=np.dot(self.mirbase_T_urbase_dkp,trans)
		#print "vec_mir_base_ur_tcp: ",vec_mir_base_ur_tcp

		# Computing the skew  matrix :
		# Translational Speed of EE is depending on the current Distance from mir_base <--> EE and the rotational speed of the mir (Cross-Product)
		skew_matrix_small=np.array([
							[0,vec_mir_base_ur_tcp[2],-vec_mir_base_ur_tcp[1]],
							[-vec_mir_base_ur_tcp[2],0,vec_mir_base_ur_tcp[0]],
							[vec_mir_base_ur_tcp[1],-vec_mir_base_ur_tcp[0],0]
							])

		#print "skew_matrix_small: \n",skew_matrix_small
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
		
		#End of MuR Jacobian Computation
		#####









		#####
		#Start of Controller with Feedforward 

		J_extended=self.extendJacobian(J)

		# compute speeds based on pathplanning
		
		target_delta_xyz=rospy.get_param("mur/target_delta_xyz")
		
		#print "target_delta_xyz: ",target_delta_xyz
		
		end_time_pathplanning=time.time()
		#end_time_pathplanning.to_sec()
		#self.start_time_pathplanning.to_sec()
		delta_T=end_time_pathplanning-self.start_time_pathplanning
		#print "delta_T: ",delta_T#/100000

		derived_cartesian_velocities=self.deriveTargetPositions(target_delta_xyz,delta_T)
		self.start_time_pathplanning=end_time_pathplanning
		

		q_dot_ur_feedForward,wheel_speeds= self.computeJointSpeeds(J_extended,derived_cartesian_velocities)

		currentPosition,urbase_R_world= self.transformOdom_to_TCP(odom,urbase_T_urtcp)

		# add delta xyz offsets to target position to generate a linear motion of tcp in odom frame
		self.target_position[0][3]=self.target_position[0][3]+target_delta_xyz[0]
		self.target_position[1][3]=self.target_position[1][3]+target_delta_xyz[1]
		self.target_position[2][3]=self.target_position[2][3]+target_delta_xyz[2]

		q_dot_ur_controller=self.positionController(self.target_position,currentPosition,urbase_R_world,J_ur)

		#print "q_dot_ur_controller: ",q_dot_ur_controller
		#print "q_dot_ur_feedForward: ",q_dot_ur_feedForward

		q_dot_ur=np.add(q_dot_ur_feedForward, q_dot_ur_controller)
		#print "q_dot_ur: ",q_dot_ur
		self.actuateJoints(q_dot_ur,wheel_speeds)
		
	def deriveTargetPositions(self,target_delta_xyz,delta_T):

		vel_x=(target_delta_xyz[0])/delta_T
		vel_y=(target_delta_xyz[1])/delta_T
		vel_z=(target_delta_xyz[2])/delta_T

		cartesian_velocities_pathplanning=[vel_x,vel_y,vel_z,0,0,0]

		print "cartesian_velocities_pathplanning: ",cartesian_velocities_pathplanning



		return cartesian_velocities_pathplanning
	
	def computeJointSpeeds(self,J_extended,derived_cartesian_velocities):

		#cartesian_velocities=rospy.get_param("mur/cartesian_velocities")
		cartesian_velocities=derived_cartesian_velocities

		cmd_wheel_velocities=rospy.get_param("mur/ramp_cmd_wheel_velocities")#np.array([1.45,1.51])

		X=np.concatenate((cartesian_velocities,cmd_wheel_velocities))

		
		
		
		J_inv=np.linalg.inv(J_extended)


		

		q_dot=np.dot(J_inv,X)
		


		
		joint_speeds_list=[q_dot[0,0],q_dot[0,1],q_dot[0,2],q_dot[0,3],q_dot[0,4],q_dot[0,5]]
		wheel_speeds=[q_dot[0,6],q_dot[0,7]]

		#print "q: ", joint_speeds_list,wheel_speeds
		return joint_speeds_list,wheel_speeds	
	
	
	def actuateJoints(self,joint_speeds_list,wheel_speeds):
		
		#print "actuateJoints"
		
		self.cmd_vel.linear.x = 0.5 * (wheel_speeds[0] + wheel_speeds[1])*self.wheel_radius
		self.cmd_vel.angular.z = self.wheel_radius*(wheel_speeds[1] - wheel_speeds[0])/self.wheel_separation

		
		#print self.cmd_vel.linear.x,self.cmd_vel.angular.z

		self.cmd_vel_publisher.publish(self.cmd_vel)		
		self.robot.publish('speedj(['+str(joint_speeds_list[0])+','+str(joint_speeds_list[1])+','+str(joint_speeds_list[2])+','+str(joint_speeds_list[3])+','+str(joint_speeds_list[4])+','+str(joint_speeds_list[5])+'], 0.2, 0.5)')
		
	

	def positionController(self,target_position,currentPosition,urbase_R_world,J_ur):

		#print "Controller target_position: ",target_position[0][3],target_position[1][3]
		#print "currentPosition: ",currentPosition[0][3],currentPosition[1][3]

		

		current_orientation=tf.transformations.euler_from_matrix(currentPosition[:-1,:-1],axes='sxyz')
		target_orientation=tf.transformations.euler_from_matrix(target_position[:-1,:-1],axes='sxyz')
		

		# compute controller errors in 6-axis
		error_x =target_position[0][3] - currentPosition[0][3]
		error_y =target_position[1][3] - currentPosition[1][3]
		error_z =target_position[2][3] - currentPosition[2][3]
		error_a =target_orientation[0] - current_orientation[0]
		error_b =target_orientation[1] - current_orientation[1]
		error_c =target_orientation[2] - current_orientation[2]

		error_tcp=np.array([error_x,error_y,error_z])

		rotated_errors=np.dot(urbase_R_world,error_tcp)

		#print "rotated_errors: ",rotated_errors


		# adjust orientation error, since it is periodic
		if error_a< -0.8*2*math.pi:
			error_a=error_a+2*math.pi
		elif error_a> 0.8*2*math.pi:
			error_a=error_a-2*math.pi

		if error_b< -0.8*2*math.pi:
			error_b=error_b+2*math.pi
		elif error_b> 0.8*2*math.pi:
			error_b=error_b-2*math.pi

		if error_c< -0.8*2*math.pi:
			error_c=error_c+2*math.pi
		elif error_c> 0.8*2*math.pi:
			error_c=error_c-2*math.pi


		# I: update error sum stack
		self.error_sum_x_array.insert(0,rotated_errors[0])
		self.error_sum_y_array.insert(0,rotated_errors[1])
		self.error_sum_z_array.insert(0,rotated_errors[2])
		
		if len(self.error_sum_x_array)>self.e_sum_buffer_length:
			self.error_sum_x_array.pop(self.e_sum_buffer_length)
		if len(self.error_sum_y_array)>self.e_sum_buffer_length:
			self.error_sum_y_array.pop(self.e_sum_buffer_length)
		if len(self.error_sum_z_array)>self.e_sum_buffer_length:
			self.error_sum_z_array.pop(self.e_sum_buffer_length)

		#print "Error x-y: ",error_x,"\t",error_y

		e_sum_x=sum(self.error_sum_x_array)
		e_sum_y=sum(self.error_sum_y_array)
		e_sum_z=sum(self.error_sum_z_array)

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

		e_sum_z_threshold=10
		# LIMIT e_sum_z:
		if e_sum_z > e_sum_z_threshold:
			e_sum_z =e_sum_z_threshold
		elif e_sum_z < -e_sum_z_threshold:
			e_sum_z =-e_sum_z_threshold

		#print "e_sum_x: ",e_sum_x,"e_sum_y: ",e_sum_y





		#UPDATING THE DIFF_ERROR STACK
		self.prev_error_x.insert(0,rotated_errors[0])
		self.prev_error_y.insert(0,rotated_errors[1])
		self.prev_error_z.insert(0,rotated_errors[2])
		
		if len(self.prev_error_x)>self.e_diff_buffer_size:
			self.prev_error_x.pop(self.e_diff_buffer_size)
		if len(self.prev_error_y)>self.e_diff_buffer_size:
			self.prev_error_y.pop(self.e_diff_buffer_size)
		if len(self.prev_error_z)>self.e_diff_buffer_size:
			self.prev_error_z.pop(self.e_diff_buffer_size)

		#print prev_error_x
		#print prev_error_y



		error_diff_x=-(rotated_errors[0]-self.prev_error_x[len(self.prev_error_x)-1])
		error_diff_y=-(rotated_errors[1]-self.prev_error_y[len(self.prev_error_y)-1])
		error_diff_z=-(rotated_errors[2]-self.prev_error_z[len(self.prev_error_z)-1])

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

		error_diff_z_threshold=10
		# LIMIT error_diff_z:
		if error_diff_z > error_diff_z_threshold:
			error_diff_z =error_diff_z_threshold
		elif error_diff_z < -error_diff_z_threshold:
			error_diff_z =-error_diff_z_threshold



		#Controller Equaition - 2-Dimensional
		speedx= self.K_P * -rotated_errors[0] - self.K_I*e_sum_x + self.K_D*error_diff_x
		speedy= self.K_P * -rotated_errors[1] - self.K_I*e_sum_y + self.K_D*error_diff_y
		speedz= self.K_P * rotated_errors[2]# - self.K_I*e_sum_z + self.K_D*error_diff_z
		speeda= self.K_P_O * error_a *0
		speedb= self.K_P_O * error_b *0
		speedc= self.K_P_O * error_c

		

		# LIMIT speedx:
		if speedx > self.speed_threshold:
			speedx =self.speed_threshold
		elif speedx < -self.speed_threshold:
			speedx =-self.speed_threshold

		# LIMIT speedy:
		if speedy > self.speed_threshold:
			speedy =self.speed_threshold
		elif speedy < -self.speed_threshold:
			speedy =-self.speed_threshold

		# LIMIT speedz:
		if speedz > self.speed_threshold:
			speedz =self.speed_threshold
		elif speedz < -self.speed_threshold:
			speedz =-self.speed_threshold

		# LIMIT speeda:
		if speeda > self.speedO_threshold:
			speeda =self.speedO_threshold
		elif speeda < -self.speedO_threshold:
			speeda =-self.speedO_threshold

		# LIMIT speedb:
		if speedb > self.speedO_threshold:
			speedb =self.speedO_threshold
		elif speedb < -self.speedO_threshold:
			speedb =-self.speedO_threshold

		# LIMIT speedc:
		if speedc > self.speedO_threshold:
			speedc =self.speedO_threshold
		elif speedc < -self.speedO_threshold:
			speedc =-self.speedO_threshold

		cartesian_velocities=[speedx,speedy,speedz,speeda,speedb,speedc]

		#print "cartesian_velocities: ",cartesian_velocities

		J_inv=np.linalg.inv(J_ur)

		q_dot_ur=np.dot(J_inv,cartesian_velocities)





		return [q_dot_ur[0,0],q_dot_ur[0,1],q_dot_ur[0,2],q_dot_ur[0,3],q_dot_ur[0,4],q_dot_ur[0,5]]

	def transformOdom_to_TCP(self,mir_pose,urbase_T_urtcp):	

		mir_trans=[mir_pose.pose.pose.position.x,mir_pose.pose.pose.position.y,mir_pose.pose.pose.position.z]
		mir_rot_quat=[mir_pose.pose.pose.orientation.x,mir_pose.pose.pose.orientation.y,mir_pose.pose.pose.orientation.z,mir_pose.pose.pose.orientation.w]
		#print "mir_pose: ",mir_pose.orientation
		world_T_mirbase=tf.transformations.quaternion_matrix(mir_rot_quat)
		world_T_mirbase[0][3]=mir_trans[0]
		world_T_mirbase[1][3]=mir_trans[1]
		world_T_mirbase[2][3]=mir_trans[2]

		

		mirbase_T_urtcp=np.dot(self.mirbase_T_urbase_dkp,urbase_T_urtcp)

		#mirbase_T_urtcp_1=np.dot(self.R_z_180,mirbase_T_urtcp)

		#print "mirbase_T_urtcp: ",mirbase_T_urtcp[0][3],mirbase_T_urtcp[1][3]			
		
		world_T_urtcp=np.dot(world_T_mirbase,mirbase_T_urtcp)

		#print "world_T_urtcp: ",world_T_urtcp[0][3],world_T_urtcp[1][3]



		world_T_urbase=np.dot(world_T_mirbase,self.mirbase_T_urbase_dkp)
		urbase_R_world=np.linalg.inv(world_T_urbase[:-1,:-1])

		return world_T_urtcp,urbase_R_world


	def messageFilter(self):

		var1=raw_input("Go on: ")
		#self.robot.publish('movej('+str(rospy.get_param("arm/null_space_init_j"))+', a=1.2, v=0.5, t=0, r=0)')
		break_loop=True
		while(break_loop):
			rospy.sleep(0.01)
			msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
			break_loop=msg.is_program_running



		self.target_position,self.urbase_R_world_init=self.transformOdom_to_TCP(self.mir_pose_init,self.urbase_T_urtcp_init)	 #initialize this as current position
		print "target_position: ",self.target_position

		# Message Filter Subscriber

		joint_states_sub = message_filters.Subscriber('/joint_states', JointState)
		odom_sub = message_filters.Subscriber('/odom', Odometry)
		wheel_vel_sub = message_filters.Subscriber('/mir/wheel_velocity', wheel_vel)

		ts = message_filters.ApproximateTimeSynchronizer([joint_states_sub,odom_sub,wheel_vel_sub], 10,55)
		
		ts.registerCallback(self.callback)
		rospy.spin()

		self.robot.publish('Stopj(2)')

if __name__ == '__main__':

	try:		
		J_pub=JacobianPublisher()
		J_pub.messageFilter()
		
	except rospy.ROSInterruptException:
		pass