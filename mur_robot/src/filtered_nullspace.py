#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import message_filters

#from message_filters import TimeSynchronizer, Subscriber
def tss_callback(robot_pose1,joint_states1):

	print robot_pose1

	



def filteredCallback(msg):

	global listener

	global last_publ_time

	

	print msg

	#listener = tf.TransformListener()
	listener.waitForTransform('/base','/tool0_controller',rospy.Time(), rospy.Duration(5))
	(trans,rot_quat) = listener.lookupTransform('/base', '/tool0_controller', rospy.Time(0))

	
	
	current_time=rospy.rostime.get_time()

	diff_time=current_time-last_publ_time

	#print "diff_time: ", diff_time

	#print "current_time: ",current_time

	if diff_time>0.005:
		print "diff_time: ",diff_time
		last_publ_time=rospy.rostime.get_time()

	#if 
	

	#print "trans: ",trans,"rot: ", rot_quat, "msg: ",msg

def filter():
	global listener
	listener = tf.TransformListener()

	global last_publ_time
	last_publ_time=rospy.rostime.get_time()
	rospy.sleep(0.5)

	#mir_pose = rospy.wait_for_message('/robot_pose', Pose)

	#rospy.Subscriber("robot_pose", Pose, controller,queue_size=1)

	sub1 = message_filters.Subscriber("/robot_pose", Pose)
	#sub2 = message_filters.Subscriber("/joint_states", JointState)

	sub1.registerCallback(filteredCallback)
	#sub2.registerCallback(filteredCallback)

	



	#tss = message_filters.TimeSequencer([sub1,sub2],10)
	#tss.registerCallback(tss_callback)

	rospy.spin()

	


if __name__ == '__main__':
	rospy.init_node('filter_node', anonymous=True)

	filter()
	
	

	
