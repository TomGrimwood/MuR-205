#!/usr/bin/env python

import rospy
import time
import math
from std_msgs.msg import String

def controller():
    
    pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    
    var=raw_input("Go on: ")
    rospy.set_param('break',1)
    while rospy.get_param('break')==1:
        rospy.sleep(0.01)
        #print "time:",time.time()

        speedx=math.sin(time.time())/8
        speedy=math.cos(time.time())/8
        print speedy
        

        #rospy.loginfo(command)
    
        pub.publish('speedl(['+str(speedx)+','+str(speedy)+',0,0,0,0], 0.5, 0.5)')

    
    
    pub.publish('Stopj(2)')
    

    #rostopic pub /ur_driver/URScript std_msgs/String "data: 'movej([3.1936185359954834, -0.9609182516681116, -2.219588104878561, -1.5329936186419886, 1.5752031803131104, 1.4252062320709229], a=1.4, v=1.05, t=0, r=0)'"





if __name__ == '__main__':
    rospy.init_node('velocity_controller_node', anonymous=True)
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
