#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState


    
def getJointPoseUR():

    msg = rospy.wait_for_message('joint_states', JointState)
    print msg.position
    rospy.set_param('/mur205/arm/currentJointPose',msg.position)
    return msg.position

    
if __name__ == '__main__':
    rospy.init_node('JointPose_subscriber', anonymous=True)
    Pose1_j=getJointPoseUR()