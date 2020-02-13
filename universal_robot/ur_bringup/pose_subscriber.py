#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
from std_msgs.msg import String


    
def getCartesianPoseUR():
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

    currentPose=[trans[0],trans[1],trans[2],euler[0],euler[1],euler[2]]
    #print "currentPose: ", currentPose

    rospy.set_param('/mur205/arm/currentCartesianPose',currentPose)
    return currentPose


if __name__ == '__main__':
    rospy.init_node('getCartesianPoseUR_Node', anonymous=True)
    Pose1=getCartesianPoseUR()
    print Pose1