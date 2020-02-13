#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist


def callback(data):
    print "\n\nX: ",data.wrench.force.x
    print "Y: ",data.wrench.force.y

    velocity_command=Twist()
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=4)

  

    velocity_command.linear.x = data.wrench.force.x * 0.01
    velocity_command.angular.z = data.wrench.force.y * 0.01

    if velocity_command.linear.x < abs(0.01):
        velocity_command.linear.x =0
    elif velocity_command.linear.x > 0.5:
        velocity_command.linear.x =0.5
    elif velocity_command.linear.x<-0.5:
        velocity_command.linear.x=-0.5

    if velocity_command.angular.z < abs(0.01):
        velocity_command.angular.z =0
    elif velocity_command.angular.z > 0.5:
        velocity_command.angular.z =0.5
    elif velocity_command.angular.z<-0.5:
        velocity_command.angular.z=-0.5

    #print "\n\nX_vel: ",velocity_command.linear.x 
    #print "Z_rot_vel: ",velocity_command.angular.z,"\n\n\n\n"



    velocity_publisher.publish(velocity_command)

    
def listener():

    
    rospy.init_node('guide_robot_node', anonymous=True)
    

    velocity_command=Twist()
    velocity_command.linear.x = 0
    velocity_command.linear.y = 0
    velocity_command.linear.z = 0
    velocity_command.angular.x = 0
    velocity_command.angular.y = 0
    velocity_command.angular.z = 0


    #velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=4)
    #print "wait_for message to be publsihed"
    rospy.Subscriber("/wrench", WrenchStamped, callback)
    
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
