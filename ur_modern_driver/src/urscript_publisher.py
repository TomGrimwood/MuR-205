#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/URScript', String, queue_size=1)
    rospy.init_node('urscript_publisher_node', anonymous=True)
    command = "data: 'movel(p[-0.254362,0.09730,0.230,3.1415,0,0], a=1.2, v=0.25, t=0, r=0)'" 
    rospy.loginfo(command)
    pub.publish(command)





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
