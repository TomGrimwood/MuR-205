#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('target_publisher_node', anonymous=True)

    target_x=rospy.get_param("target_x")
    target_y=rospy.get_param("target_y")
    
    rospy.set_param("break",1)

    frequency=10#input("Speed: ")
    steps_x=0.001#input("steps_x: ")
    steps_y=0.005#input("steps_x: ")

    rate = rospy.Rate(frequency) # 10hz
    while not rospy.is_shutdown() and rospy.get_param("break")==1:
        rospy.set_param("target_x",target_x)
        rospy.set_param("target_y",target_y)
        target_x+=steps_x
        target_y+=steps_y
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
