#!/usr/bin/env python
import rospy
import sys
import time
import tf
import numpy as np
from scipy.optimize import minimize
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
	rospy.set_param("mur/cmd_wheel_velocities",[0,0])

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
	# Since dkp kinematics and moveit coordinate frames are rotated about pi, this defines the other transformation, which is needed for the skew_matrix, as seen below
	mirbase_T_urbase_dkp=tf.transformations.euler_matrix(0,0,(math.pi - 0.0154892745077))# Since get_jacobian Frame is rotated about pi, T looks like this
	mirbase_T_urbase_dkp[0][3]=t_mir_ur[0]
	mirbase_T_urbase_dkp[1][3]=t_mir_ur[1]
	mirbase_T_urbase_dkp[2][3]=t_mir_ur[2]

	mirbase_R_urbase=mirbase_T_urbase[:-1,0:-1]

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

	


	
	

	try:
		print "start"
		group = moveit_commander.MoveGroupCommander('manipulator')
	except Exception as e: 
		print(e)
	joint_values=np.array([])



	def optimizeManipulability(self,joint_states_array):
		

		# Criteria for euclidean distance base_tcp
		def h_1(joint_states_array):
			link1=0.425#m
			link2=0.39225#
			delta_distance=0.3
			return math.sqrt((math.sqrt((math.cos(joint_states_array[0])*(-link1*math.cos(joint_states_array[1])-link2*math.cos(joint_states_array[1]+joint_states_array[2])))**2)-delta_distance)**2)

			#return (-link1*math.cos(joint_states_array[1])-link2*math.cos(joint_states_array[1]+joint_states_array[2]))-delta_distance

		#optimization criteria for base joint:
		def h(joint_states_array):

			
			return 0.5 *(joint_states_array[0]-(math.pi)*110/180)**2 + 0.5*(joint_states_array[1]-(math.pi)*-90/180)**2
		#print "Distance: ",h_1(joint_states_array)
		#minimization:
		q_0=minimize(h_1,[joint_states_array[0],joint_states_array[1],joint_states_array[2]],method='Nelder-Mead',tol=10)
		#print "Target q_0: ", q_0.x[0]

		#print "Optimum: q_0: ",q_0

		#q_0=math.pi

		return q_0

	def computeJointSpeeds(self,J,joint_states_array):

		
		

		cartesian_velocities=rospy.get_param("mur/cartesian_velocities")
		#print "cartesian_velocities: ",cartesian_velocities
		#try:
		q_0=self.optimizeManipulability(joint_states_array)
		#except:
		#print "Optimization Failed!"
		stepsize=100

		q_dot_0=np.array([-stepsize*(joint_states_array[0]-q_0.x[0]) ,-stepsize*(joint_states_array[1]-q_0.x[1]),-stepsize*(joint_states_array[2]-q_0.x[2]),0,0,0,0,0])

		print "q_dot_0 l/r: ",q_dot_0[6],q_dot_0[7]


		
		
		
		
		J_pinv=np.linalg.pinv(J)
		print "J_pinv: \n",J_pinv

		q_dot_1=np.dot(J_pinv,cartesian_velocities)

		I=np.eye(8)
		#print "I: ",I

		J_=np.dot(J_pinv,J)

		# A. de Luca et al. Equation (5)
		print "I -J_: \n",I - J_
		#print "proj. grad. :",np.dot((I - J_),q_dot_0)
		#print "q_dot_0: ",q_dot_0

		
		q_dot= q_dot_1 + np.dot((I - J_),q_dot_0)

		#print "q_dot: ",q_dot



		
		joint_speeds_list=(q_dot[0,0],q_dot[0,1],q_dot[0,2],q_dot[0,3],q_dot[0,4],q_dot[0,5])
		wheel_speeds=[q_dot[0,6],q_dot[0,7]]

		#print "q: ", joint_speeds_list,wheel_speeds
		return joint_speeds_list,wheel_speeds

	def actuateJoints(self,joint_speeds_list,wheel_speeds):
		
		#print "actuateJoints"
		
		self.cmd_vel.linear.x = 0.5 * (wheel_speeds[0] + wheel_speeds[1])*self.wheel_radius
		self.cmd_vel.angular.z = self.wheel_radius*(wheel_speeds[1] - wheel_speeds[0])/self.wheel_separation

		
		#print self.cmd_vel.linear.x,self.cmd_vel.angular.z

		self.cmd_vel_publisher.publish(self.cmd_vel)
		self.robot.publish('speedj(['+str(joint_speeds_list[0])+','+str(joint_speeds_list[1])+','+str(joint_speeds_list[2])+','+str(joint_speeds_list[3])+','+str(joint_speeds_list[4])+','+str(joint_speeds_list[5])+'], 0.5, 0.5)')
		


	def extendJacobian(self,J):


		#J_T=np.transpose(J)
		
		#dex_index=np.linalg.det(J*J_T)

		#print "dex_index: ",dex_index*1000


		constraint_1=np.array([[1,0,0,0,0,0,0,0]])
		constraint_2=np.array([[0,1,0,0,0,0,0,0]])
		constraint_3=np.array([[0,0,1,0,0,0,0,0]])
		constraint_4=np.array([[0,0,0,1,0,0,0,0]])
		constraint_5=np.array([[0,0,0,0,1,0,0,0]])
		constraint_6=np.array([[0,0,0,0,0,1,0,0]])

		#print "J: \n",J
		#add constraints to jacobian
		J_extended=np.concatenate((J,constraint_1),0)
		J_extended=np.concatenate((J_extended,constraint_2),0)
		J_extended=np.concatenate((J_extended,constraint_3),0)
		J_extended=np.concatenate((J_extended,constraint_4),0)
		J_extended=np.concatenate((J_extended,constraint_5),0)
		J_extended=np.concatenate((J_extended,constraint_6),0)
		

		#print "J_extended: \n",J_extended

		rank_J_extended = np.linalg.matrix_rank(J_extended)
		print "Rank of J_extended: ",rank_J_extended





		return J_extended



	def messageFilter(self):

		var1=raw_input("Go on: ")
		#self.robot.publish('movej('+str(rospy.get_param("arm/null_space_init_j"))+', a=1.2, v=0.5, t=0, r=0)')
		
		break_loop=True
		
		while(break_loop):
			rospy.sleep(0.01)
			msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
			break_loop=msg.is_program_running


		joint_states_sub = message_filters.Subscriber('/joint_states', JointState)
		odom_sub = message_filters.Subscriber('/odom', Odometry)
		wheel_vel_sub = message_filters.Subscriber('/mir/wheel_velocity', wheel_vel)
		

		ts = message_filters.ApproximateTimeSynchronizer([joint_states_sub,odom_sub,wheel_vel_sub], 10,55)
		
		ts.registerCallback(self.callback)
		rospy.spin()

		self.robot.publish('Stopj(2)')

	def callback(self,joint_states,odom,wheel_vel):
		
		##############################
		
		#COMPUTATION OF MUR JACOBIAN :

		time_start=rospy.get_time()
		


		self.listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))

		(trans,rot_quat) = self.listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

		

		joint_states_array=[joint_states.position[0],joint_states.position[1],joint_states.position[2],joint_states.position[3],joint_states.position[4],joint_states.position[5]]
		J_ur=self.group.get_jacobian_matrix(joint_states_array)

		

		dex_index=np.linalg.det(J_ur)

		u, s, vh = np.linalg.svd(J_ur)

		#print "s: ",s[5], -dex_index


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


		u_mir, s_mir, vh_mir = np.linalg.svd(J_mir)

		#print "s_mir: ",s_mir

		
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

		

		#J_extended=self.extendJacobian(J)

		# projected gradient method:
		
		joint_speeds,wheel_speeds= self.computeJointSpeeds(J,joint_states_array)

		self.actuateJoints(joint_speeds,wheel_speeds)

		time_end=rospy.get_time()

		#print "time_diff: ",time_end-time_start
		
		

		
		
		

if __name__ == '__main__':
	try:
		
		J_pub=JacobianPublisher()
		J_pub.messageFilter()
		
	except rospy.ROSInterruptException:
		pass