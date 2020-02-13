#!/usr/bin/env python
import rospy
import tf
import yaml
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose



	
def getPoseUR():
	try:
		listener = tf.TransformListener()
		listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(0.1))
		(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))
	except Exception, e:
		logger.error('Failed during tf.TransformListener : '+ str(e)+' \n\n Check if Correct Topics are being published!')
		return 0
		

	#print "trans: ",trans
	#print "rot_quat: ",rot_quat

	euler = tf.transformations.euler_from_quaternion(rot_quat)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#print "RPY: ", roll, pitch, yaw

	pose_tcp=[trans[0],trans[1],trans[2],euler[0],euler[1],euler[2]]
	#print "pose_tcp: ", pose_tcp

	msg = rospy.wait_for_message('joint_states', JointState)
	#print msg.position
	
	pose_j = msg.position
	pose_j_array=[pose_j[0],pose_j[1],pose_j[2],pose_j[3],pose_j[4],pose_j[5]]
	#print type(pose_j_array)

	
	return pose_j_array,pose_tcp


def savePoseUR(directory,pose_name,joint_pose,cartesian_pose):

	#pose_dict=dict()
	with open(directory+'/arm_poses.yaml', 'r') as stream:
		pose_dict = yaml.safe_load(stream)
	#print pose_dict

	pose_name_j='arm/'+pose_name+'_j'
	pose_name_tcp='arm/'+pose_name+'_tcp'
	new_pose_data={pose_name_j:joint_pose,pose_name_tcp:cartesian_pose}

	pose_dict.update(new_pose_data)

	
	with open(directory+'/arm_poses.yaml', 'w') as outfile:
		yaml.dump(pose_dict, outfile)


def getPoseMiR():

	mir_pose = rospy.wait_for_message('/robot_pose', Pose)
	#print "mir_pose: ",mir_pose

	return mir_pose

def savePoseMiR(directory,pose_name,pose):
	
	
	with open(directory+'/mir_poses.yaml', 'r') as stream:
		pose_dict = yaml.safe_load(stream)

	mir_pose_name='mir/'+pose_name
	
	pose_array=[pose.position.x,pose.position.y,pose.position.z,pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
	
	new_pose_data={mir_pose_name:pose_array}

	pose_dict.update(new_pose_data)

	
	with open(directory+'/mir_poses.yaml', 'w') as outfile:
		yaml.dump(pose_dict, outfile)
	


if __name__ == '__main__':
	rospy.init_node('getCartesianPoseUR_Node', anonymous=True)
	
	directory='/home/rosmatch/catkin_ws/src/mur_robot/config'

	mode=raw_input("Save Pose from\n\n UR5: (u)\nMiR: (m)")

	pose_name=raw_input("Enter Pose Name: ")

	if mode=='u':
		joint_pose, cartesian_pose =getPoseUR()
		savePoseUR(directory,pose_name,joint_pose,cartesian_pose)

	elif mode=='m':
		mir_pose=getPoseMiR()
		savePoseMiR(directory,pose_name,mir_pose)

	else:
		print "Wrong input! Terminating"


	
