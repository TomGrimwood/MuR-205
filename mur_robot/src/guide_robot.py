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

    #cartesian_velocities=[0,0,0,0,0,0]

    #rospy.set_param("mur/cartesian_velocities",cartesian_velocities)

    x_vel_threshold=0.2
    '''
    velocity_command.linear.x = data.wrench.force.x * -0.02
    velocity_command.angular.z = data.wrench.force.y * -0.02

    if abs(velocity_command.linear.x) < 0.1:

        velocity_command.linear.x =0
    if velocity_command.linear.x > x_vel_threshold:

        velocity_command.linear.x =x_vel_threshold
    elif velocity_command.linear.x<-x_vel_threshold:
        velocity_command.linear.x=-x_vel_threshold

    if abs(velocity_command.angular.z) < 0.1:
        velocity_command.angular.z =0
    if velocity_command.angular.z > 0.5:
        velocity_command.angular.z =0.5
    elif velocity_command.angular.z<-0.5:
        velocity_command.angular.z=-0.5
    '''
    cartesian_velocities=[data.wrench.force.y * -0.008,-data.wrench.force.x * -0.008,data.wrench.force.z*0.008 ,0,0,0]

    if abs(cartesian_velocities[0]) < 0.12:

        cartesian_velocities[0] =0
    elif cartesian_velocities[0] > x_vel_threshold:
        cartesian_velocities[0] =x_vel_threshold

    elif cartesian_velocities[0]<-x_vel_threshold:
        cartesian_velocities[0]=-x_vel_threshold


    if abs(cartesian_velocities[1]) < 0.12:
        cartesian_velocities[1] =0

    elif cartesian_velocities[1] > x_vel_threshold:
        cartesian_velocities[1] =x_vel_threshold

    elif cartesian_velocities[1]<-x_vel_threshold:
        cartesian_velocities[1]=-x_vel_threshold


    if abs(cartesian_velocities[2]) < 0.12:
        cartesian_velocities[2] =0

    elif cartesian_velocities[2] > x_vel_threshold:
        cartesian_velocities[2] =x_vel_threshold

    elif cartesian_velocities[2]<-x_vel_threshold:
        cartesian_velocities[2]=-x_vel_threshold


    #print "\n\nX_vel: ",velocity_command.linear.x 
    #print "Z_rot_vel: ",velocity_command.angular.z,"\n\n\n\n"

    print "cartesian_velocities: ",cartesian_velocities

    rospy.set_param("mur/cartesian_velocities",cartesian_velocities)

    #velocity_publisher.publish(velocity_command)

    
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
    cartesian_velocities=[0,0,0,0,0,0]

    rospy.set_param("mur/cartesian_velocities",cartesian_velocities)

if __name__ == '__main__':
    var=raw_input("Go: ")
    listener()
    cartesian_velocities=[0,0,0,0,0,0]

    rospy.set_param("mur/cartesian_velocities",cartesian_velocities)

