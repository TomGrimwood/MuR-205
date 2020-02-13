#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg

def talker():
    
    pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
    rospy.init_node('urscript_publisher_node', anonymous=True)
    var=raw_input("Go on: ")


    command = 'movel(p[-0.254362,0.09730,0.230,3.1415,0,0], a=1.2, v=0.25, t=0, r=0)'

    #rospy.loginfo(command)
    
    #pub.publish('movel(p[-0.254362,0.09730,0.230,3.1415,0,0], a=1.2, v=0.25, t=0, r=0)')
    #pub.publish('RG2(target_width=90)')

    cmd_str = "def rg2ProgOpen():\n"
    #cmd_str += "\ttextmsg(\"inside RG2 function called\")\n"
    cmd_str += "\ttarget_width=110\n"

    #cmd_str += buf;

    

    cmd_str += "\ttarget_force=40\n"
    cmd_str += "\tpayload=1.0\n"
    cmd_str += "\tset_payload1=False\n"
    cmd_str += "\tdepth_compensation=False\n"
    cmd_str += "\tslave=False\n"

    #cmd_str += "\ttimeout = 0\n"
    cmd_str += "\twhile get_digital_in(9) == False:\n"
    cmd_str += "\t\ttextmsg(\"inside while\")\n"
    cmd_str += "\t\tif timeout > 400:\n"
    cmd_str += "\t\t\tbreak\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\ttimeout = timeout+1\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\tend\n"
    cmd_str += "\ttextmsg(\"outside while\")\n"

    #cmd_str += "\tdef bit(input):\n"
    cmd_str += "def bit(input):\n"
    cmd_str += "\t\tmsb=65536\n"
    cmd_str += "\t\tlocal i=0\n"
    cmd_str += "\t\tlocal output=0\n"
    cmd_str += "\t\twhile i<17:\n"
    cmd_str += "\t\t\tset_digital_out(8,True)\n"
    cmd_str += "\t\t\tif input>=msb:\n"
    cmd_str += "\t\t\t\tinput=input-msb\n"
    cmd_str += "\t\t\t\tset_digital_out(9,False)\n"
    cmd_str += "\t\t\telse:\n"
    cmd_str += "\t\t\t\tset_digital_out(9,True)\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\t\tif get_digital_in(8):\n"
    cmd_str += "\t\t\t\tout=1\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\t\tset_digital_out(8,False)\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\t\tinput=input*2\n"
    cmd_str += "\t\t\toutput=output*2\n"
    cmd_str += "\t\t\ti=i+1\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\treturn output\n"
    cmd_str += "\tend\n"
    cmd_str += "\ttextmsg(\"outside bit definition\")\n"


    cmd_str += "\ttarget_width=target_width+0.0\n"
    cmd_str += "\tif target_force>40:\n"
    cmd_str += "\t\ttarget_force=40\n"
    cmd_str += "\tend\n"

    cmd_str += "\tif target_force<4:\n"
    cmd_str += "\t\ttarget_force=4\n"
    cmd_str += "\tend\n"
    cmd_str += "\tif target_width>110:\n"
    cmd_str += "\t\ttarget_width=110\n"
    cmd_str += "\tend\n"
    cmd_str += "\tif target_width<0:\n"
    cmd_str += "\t\ttarget_width=0\n"
    cmd_str += "\tend\n"
    cmd_str += "\trg_data=floor(target_width)*4\n"
    cmd_str += "\trg_data=rg_data+floor(target_force/2)*4*111\n"
    cmd_str += "\tif slave:\n"
    cmd_str += "\t\trg_data=rg_data+16384\n"
    cmd_str += "\tend\n"

    cmd_str += "\ttextmsg(\"about to call bit\")\n"
    cmd_str += "\tbit(rg_data)\n"
    cmd_str += "\ttextmsg(\"called bit\")\n"

    cmd_str += "\tif depth_compensation:\n"
    cmd_str += "\t\tfinger_length = 55.0/1000\n"
    cmd_str += "\t\tfinger_heigth_disp = 5.0/1000\n"
    cmd_str += "\t\tcenter_displacement = 7.5/1000\n"

    cmd_str += "\t\tstart_pose = get_forward_kin()\n"
    cmd_str += "\t\tset_analog_inputrange(2, 1)\n"
    cmd_str += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    cmd_str += "\t\tzangle = zscale*1.57079633-0.087266462\n"
    cmd_str += "\t\tzwidth = 5+110*sin(zangle)\n"

    cmd_str += "\t\tstart_depth = cos(zangle)*finger_length\n"


    cmd_str += "\t\tsync()\n"
    cmd_str += "\t\tsync()\n"
    cmd_str += "\t\ttimeout = 0\n"

    cmd_str += "\t\twhile get_digital_in(9) == True:\n"
    cmd_str += "\t\t\ttimeout=timeout+1\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\t\tif timeout > 20:\n"
    cmd_str += "\t\t\t\tbreak\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\ttimeout = 0\n"
    cmd_str += "\t\twhile get_digital_in(9) == False:\n"
    cmd_str += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n"
    cmd_str += "\t\t\tzangle = zscale*1.57079633-0.087266462\n"
    cmd_str += "\t\t\tzwidth = 5+110*sin(zangle)\n"
    cmd_str += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n"
    cmd_str += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n"
    cmd_str += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n"
    cmd_str += "\t\t\tif timeout > 400:\n"
    cmd_str += "\t\t\t\tbreak\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\t\ttimeout=timeout+1\n"
    cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\tnspeed = norm(get_actual_tcp_speed())\n"
    cmd_str += "\t\twhile nspeed > 0.001:\n"
    cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.033,1700)\n"
    cmd_str += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\tend\n"
    cmd_str += "\tif depth_compensation==False:\n"
    cmd_str += "\t\ttimeout = 0\n"
    cmd_str += "\t\twhile get_digital_in(9) == True:\n"
    cmd_str += "\t\t\ttimeout = timeout+1\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\t\tif timeout > 20:\n"
    cmd_str += "\t\t\t\tbreak\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\ttimeout = 0\n"
    cmd_str += "\t\twhile get_digital_in(9) == False:\n"
    cmd_str += "\t\t\ttimeout = timeout+1\n"
    cmd_str += "\t\t\tsync()\n"
    cmd_str += "\t\t\tif timeout > 400:\n"
    cmd_str += "\t\t\t\tbreak\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\tend\n"
    cmd_str += "\tif set_payload1:\n"
    cmd_str += "\t\tif slave:\n"
    cmd_str += "\t\t\tif get_analog_in(3) < 2:\n"
    cmd_str += "\t\t\t\tzslam=0\n"
    cmd_str += "\t\t\telse:\n"
    cmd_str += "\t\t\t\tzslam=payload\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\telse:\n"
    cmd_str += "\t\t\tif get_digital_in(8) == False:\n"
    cmd_str += "\t\t\t\tzmasm=0\n"
    cmd_str += "\t\t\telse:\n"
    cmd_str += "\t\t\t\tzmasm=payload\n"
    cmd_str += "\t\t\tend\n"
    cmd_str += "\t\tend\n"
    cmd_str += "\t\tzsysm=0.0\n"
    cmd_str += "\t\tzload=zmasm+zslam+zsysm\n"
    cmd_str += "\t\tset_payload(zload)\n"
    cmd_str += "\tend\n"

    print "Program({})".format(cmd_str)

    



    pub.publish("Program({})".format(cmd_str))
    msg='Program Robot Program RG2(100)'
    
    #pub.publish('set_digital_out(2,True)')
    #

    '''
    pub.publish('movej('+str(rospy.get_param("arm/null_space_init_j"))+', a=1.2, v=0.25, t=0, r=0)')
    break_loop=True
    while(break_loop):
      rospy.sleep(0.001)
      msg=rospy.wait_for_message('/ur_driver/robot_mode_state',RobotModeDataMsg)
      break_loop=msg.is_program_running
    '''
    
    print "Done"


    #rostopic pub /ur_driver/URScript std_msgs/String "data: 'movej([3.1936185359954834, -0.9609182516681116, -2.219588104878561, -1.5329936186419886, 1.5752031803131104, 1.4252062320709229], a=1.4, v=1.05, t=0, r=0)'"





if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
