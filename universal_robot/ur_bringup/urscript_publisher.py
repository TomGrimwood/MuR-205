#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    
    pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    rospy.init_node('urscript_publisher_node', anonymous=True)
    var=raw_input("Go on: ")


    command = 'movel(p[-0.254362,0.09730,0.230,3.1415,0,0], a=1.2, v=0.25, t=0, r=0)'

    rospy.loginfo(command)
    
    pub.publish('movel(p[-0.254362,0.09730,0.230,3.1415,0,0], a=1.2, v=0.25, t=0, r=0)')
    print "Done"

    #rostopic pub /ur_driver/URScript std_msgs/String "data: 'movej([3.1936185359954834, -0.9609182516681116, -2.219588104878561, -1.5329936186419886, 1.5752031803131104, 1.4252062320709229], a=1.4, v=1.05, t=0, r=0)'"





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
