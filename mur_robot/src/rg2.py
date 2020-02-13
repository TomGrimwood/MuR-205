#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from ur_msgs.msg import RobotModeDataMsg

def talker():
	
	pub = rospy.Publisher('/ur_driver/URScript', String, queue_size=1)
	rospy.init_node('urscript_publisher_node', anonymous=True)
	var=raw_input("Go on: ")

cmd_str += "def unnamed():\n"
cmd_str += "\tset_standard_analog_input_domain(0, 1)\n"
cmd_str += "\tset_standard_analog_input_domain(1, 1)\n"
cmd_str += "\tset_tool_analog_input_domain(0, 1)\n"
cmd_str += "\tset_tool_analog_input_domain(1, 1)\n"
cmd_str += "\tset_analog_outputdomain(0, 0)\n"
cmd_str += "\tset_analog_outputdomain(1, 0)\n"
cmd_str += "\tset_input_actions_to_default()\n"
cmd_str += "\tset_tcp(p[0.0,0.0,0.0,0.0,0.0,0.0])\n"
cmd_str += "\tset_payload(0.0)\n"
cmd_str += "\tset_gravity([0.0, 0.0, 9.82])\n"
cmd_str += "\tset_tool_voltage(12)\n"
cmd_str += "\tset_safety_mode_transition_hardness(1)\n"
cmd_str += "\tglobal i_var_3=p[0.01, 0.05, 0, 0, 0, 0]\n"
cmd_str += "\tglobal i_var_2=p[0.09, 0.03, 0, 0, 0, 0]\n"
cmd_str += "\tglobal i_var_1=p[0.04, 0.09, 0, 0, 0, 0]\n"
gcmd_str += "\tlobal Plane_1=p[0.4117770987147006,-0.4990341275909978,0.14907993082148976,1.570611150510273,5.470302593228572E-4,-3.5212753610990396E-4]\n"
cmd_str += "\tglobal labor_test=p[0.423092607896834,-0.14299498849910064,0.3655508045003514,-1.8153926306203706E-4,-1.2287633123027537E-4,-1.2040001162655254E-4]\n"
cmd_str += "\t# begin: URCap Installation Node\n"
cmd_str += "\t#   Source: RG - On Robot, 1.10.1, OnRobot A/S\n"
cmd_str += "\t#   Type:  RG Konfiguration\n"
cmd_str += "\tglobal measure_width=0\n"
cmd_str += "\tglobal grip_detected=False\n"
cmd_str += "\tglobal lost_grip=False\n"
cmd_str += "\tglobal zsysx=0\n"
cmd_str += "\tglobal zsysy=0\n"
cmd_str += "\tglobal zsysz=0.06935\n"
cmd_str += "\tglobal zsysm=0.7415\n"
cmd_str += "\tglobal zmasx=0\n"
cmd_str += "\tglobal zmasy=-0\n"
cmd_str += "\tglobal zmasz=0.18659\n"
cmd_str += "\tglobal zslax=0\n"
cmd_str += "\tglobal zslay=0\n"
cmd_str += "\tglobal zslaz=0\n"
cmd_str += "\tglobal zmasm=0\n"
cmd_str += "\tglobal zslam=0\n"
cmd_str += "\tglobal zslatcp=p[0,0,0,0,0,0]\n"
cmd_str += "\tglobal zmastcp=p[0,0,0.18659,0,-0,-3.14159]\n"
cmd_str += "\tthread lost_grip_thread():\n"
cmd_str += "\twhile True:\n"
cmd_str += "\tset_tool_voltage(24)\n"
cmd_str += "\t  	if True ==get_digital_in(9):\n"
cmd_str += "\t  		sleep(0.024)\n"
cmd_str += "\t  		if True == grip_detected:\n"
cmd_str += "\t  			if False == get_digital_in(8):\n"
cmd_str += "\t  				grip_detected=False\n"
cmd_str += "\t  				lost_grip=True\n"
cmd_str += "\t  			end\n"
cmd_str += "\t  		end\n"
cmd_str += "\t  	set_tool_analog_input_domain(0, 1)\n"
cmd_str += "\t  	set_tool_analog_input_domain(1, 1)\n"
cmd_str += "\t  	zscale = (get_analog_in(2)-0.026)/2.9760034\n"
cmd_str += "\t  	zangle = zscale*1.57079633+-0.08726646\n"
cmd_str += "\t  	zwidth = 5.0+110*sin(zangle)\n"
cmd_str += "\t  	global measure_width = (floor(zwidth*10))/10-9.2\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	sync()\n"
cmd_str += "\t  end\n"
cmd_str += "\t  end\n"
cmd_str += "\t  lg_thr = run lost_grip_thread()\n"

cmd_str += "\t  def RG2(target_width=110, target_force=40, payload=0.0, set_payload=False, depth_compensation=False, slave=False):\n"
cmd_str += "\t  	set_tcp(p[0,0,0.18659,0,-0,-3.14159])\n"
cmd_str += "\t  	grip_detected=False\n"
cmd_str += "\t  	if slave:\n"
cmd_str += "\t  		slave_grip_detected=False\n"
cmd_str += "\t  	else:\n"
cmd_str += "\t  		master_grip_detected=False\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	timeout = 0\n"
cmd_str += "\t  	timeout_limit = 750000\n"
cmd_str += "\t  	while get_digital_in(9) == False:\n"
cmd_str += "\t  	  if timeout > timeout_limit:\n"
cmd_str += "\t  		break\n"
cmd_str += "\t  	  end\n"
cmd_str += "\t  	  timeout = timeout+1\n"
cmd_str += "\t  	  sync()\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	def bit(input):\n"
cmd_str += "\t  	  msb=65536\n"
cmd_str += "\t  	  local i=0\n"
cmd_str += "\t  	  local output=0\n"
cmd_str += "\t  	  while i<17:\n"
cmd_str += "\t  		set_digital_out(8,True)\n"
cmd_str += "\t  		if input>=msb:\n"
cmd_str += "\t  		  input=input-msb\n"
cmd_str += "\t  		  set_digital_out(9,False)\n"
cmd_str += "\t  		else:\n"
cmd_str += "\t  		  set_digital_out(9,True)\n"
cmd_str += "\t  		end\n"
cmd_str += "\t  		if get_digital_in(8):\n"
cmd_str += "\t  		  out=1\n"
cmd_str += "\t  		end\n"
cmd_str += "\t  		sync()\n"
cmd_str += "\t  		set_digital_out(8,False)\n"
cmd_str += "\t  		sync()\n"
cmd_str += "\t  		input=input*2\n"
cmd_str += "\t  		output=output*2\n"
cmd_str += "\t  		i=i+1\n"
cmd_str += "\t  	  end\n"
cmd_str += "\t  	  return output\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	target_width=target_width+9.2\n"
cmd_str += "\t  	if target_force>40:\n"
cmd_str += "\t  	target_force=40\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	if target_force<3:\n"
cmd_str += "\t  	target_force=3\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	if target_width>110:\n"
cmd_str += "\t  	target_width=110\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	if target_width<0:\n"
cmd_str += "\t  	target_width=0\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	rg_data=floor(target_width)*4\n"
cmd_str += "\t  	rg_data=rg_data+floor(target_force/2)*4*111\n"
cmd_str += "\t  	rg_data=rg_data+32768\n"
cmd_str += "\t  	if slave:\n"
cmd_str += "\t  	rg_data=rg_data+16384\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	bit(rg_data)\n"
cmd_str += "\t  	if slave==False:\n"
cmd_str += "\t  	t_w_rg=pose_trans(get_actual_tool_flange_pose(), zmastcp)\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	if slave:\n"
cmd_str += "\t  	t_w_rg=pose_trans(get_actual_tool_flange_pose(), zslatcp)\n"
cmd_str += "\t  	end\n"
cmd_str += "\t	t_rg_w=pose_inv(t_w_rg)\n"
cmd_str += "\t  	if depth_compensation:\n"
cmd_str += "\t  	finger_length = 55.0/1000\n"
cmd_str += "\t  	finger_heigth_disp = 5.0/1000\n"
cmd_str += "\t  	center_displacement = 7.5/1000\n"
cmd_str += "\t\n  
cmd_str += "\t  	start_pose = get_forward_kin()\n"
cmd_str += "\t  	set_analog_inputrange(2, 1)\n"
cmd_str += "\t  	zscale = (get_analog_in(2)-0.026)/2.9760034\n"
cmd_str += "\t  	zangle = zscale*1.57079633+-0.08726646\n"
cmd_str += "\t  	zwidth = 5.0+110*sin(zangle)\n"
cmd_str += "\t  
cmd_str += "\t  	start_depth = cos(zangle)*finger_length\n"
cmd_str += "\t  
cmd_str += "\t  	sleep(0.016)\n"
cmd_str += "\t  	timeout = 0\n"
cmd_str += "\t  	while get_digital_in(9) == True:\n"
cmd_str += "\t  	  timeout=timeout+1\n"
cmd_str += "\t  	  sleep(0.008)\n"
cmd_str += "\t  	  if timeout > 20:\n"
cmd_str += "\t  		break\n"
cmd_str += "\t  	  end\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	timeout = 0\n"
cmd_str += "\t  	timeout_limit = 750000\n"
cmd_str += "\t  	compensation_depth = 0\n"
cmd_str += "\t  	while get_digital_in(9) == False:\n"
cmd_str += "\t  	  zscale = (get_analog_in(2)-0.026)/2.9760034\n"
cmd_str += "\t  	  zangle = zscale*1.57079633+-0.08726646\n"
cmd_str += "\t  	  zwidth = 5.0+110*sin(zangle)\n"
cmd_str += "\t  	  measure_depth = cos(zangle)*finger_length\n"
cmd_str += "\t  	  compensation_depth = (measure_depth - start_depth)\n"
cmd_str += "\t  	  target_pose =pose_add(start_pose,pose_trans(pose_trans(t_w_rg, p[0,0,-compensation_depth,0,0,0]),t_rg_w))\n"
cmd_str += "\t  \n"
cmd_str += "\t  	  if timeout > timeout_limit:\n"
cmd_str += "\t  		break\n"
cmd_str += "\t  	  end\n"
cmd_str += "\t  	  timeout=timeout+1\n"
  	#  servoj(get_inverse_kin(target_pose), t=0.008, lookahead_time=0.033, gain=1500)
  	#  textmsg(point_dist(target_pose, get_forward_kin()))
  	#end
  	#textmsg("end gripper move!!!!!")
  	#nspeedthr = 0.001
  	#nspeed = norm(get_actual_tcp_speed())
  	#while nspeed > nspeedthr:
  	#  servoj(get_inverse_kin(target_pose), t=0.008, lookahead_time=0.033, gain=1500)
  	#  nspeed = norm(get_actual_tcp_speed())
  	#  textmsg(point_dist(target_pose, get_forward_kin()))
  	#end
cmd_str += "\t  	servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)\n"
cmd_str += "\t  	if point_dist(target_pose, get_forward_kin()) > 0.005:\n"
cmd_str += "\t  	popup("Lower grasping force or max width",title="RG-lag threshold exceeded", warning=False, error=False, blocking=False)\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	end\n"
cmd_str += "\t  	act_comp_pose = p[0,0,0,0,0,0]\n"
cmd_str += "\t  	while norm(act_comp_pose) < norm(compensation_depth)-0.0002:
cmd_str += "\t  	servoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)
cmd_str += "\t  	act_comp_pose = pose_trans(pose_inv(start_pose),get_forward_kin())
cmd_str += "\t  	end
cmd_str += "\t  	stopj(2)
cmd_str += "\t  	end
cmd_str += "\t  	if depth_compensation==False:
cmd_str += "\t  	timeout = 0
cmd_str += "\t  	timeout_count=20*0.008/0.008
cmd_str += "\t  	while get_digital_in(9) == True:
cmd_str += "\t  	  timeout = timeout+1
cmd_str += "\t  	  sync()
cmd_str += "\t  	  if timeout > timeout_count:
cmd_str += "\t  		break
cmd_str += "\t  	  end
cmd_str += "\t  	end
cmd_str += "\t  	timeout = 0
cmd_str += "\t  	timeout_limit = 750000
cmd_str += "\t  	while get_digital_in(9) == False:
cmd_str += "\t  	  timeout = timeout+1
cmd_str += "\t  	  sync()
cmd_str += "\t  	  if timeout > timeout_limit:
cmd_str += "\t  		break
cmd_str += "\t  	  end
cmd_str += "\t  	end
cmd_str += "\t  	end
cmd_str += "\t  	sleep(0.024)
cmd_str += "\t  	if set_payload:
cmd_str += "\t  	if slave:
cmd_str += "\t  	if get_analog_in(3)/0.5952007 < 1.42:
cmd_str += "\t  	zslam=0
cmd_str += "\t  	else:
cmd_str += "\t  	zslam=payload
cmd_str += "\t  	end
cmd_str += "\t  	else:
cmd_str += "\t  	if get_digital_in(8) == False:
cmd_str += "\t  	zmasm=0
cmd_str += "\t  	else:
cmd_str += "\t  	zmasm=payload
cmd_str += "\t  	end
cmd_str += "\t  	end
cmd_str += "\t  	zload=zmasm+zslam+zsysm
cmd_str += "\t  	set_payload(zload,[(zsysx*zsysm+zmasx*zmasm+zslax*zslam)/zload,(zsysy*zsysm+zmasy*zmasm+zslay*zslam)/zload,(zsysz*zsysm+zmasz*zmasm+zslaz*zslam)/zload])
cmd_str += "\t  	end
cmd_str += "\t  	master_grip_detected=False
cmd_str += "\t  	master_lost_grip=False
cmd_str += "\t  	slave_grip_detected=False
cmd_str += "\t  	slave_lost_grip=False
cmd_str += "\t  	if True == get_digital_in(8):
cmd_str += "\t  		master_grip_detected=True
cmd_str += "\t  	end
cmd_str += "\t  	if get_analog_in(3)/0.5952007>1.97:
cmd_str += "\t  		slave_grip_detected=True
cmd_str += "\t  	end
cmd_str += "\t  	grip_detected=False
cmd_str += "\t  	lost_grip=False
cmd_str += "\t  	if True == get_digital_in(8):
cmd_str += "\t  		grip_detected=True
cmd_str += "\t  	end
cmd_str += "\t  	zscale = (get_analog_in(2)-0.026)/2.9760034
cmd_str += "\t  	zangle = zscale*1.57079633+-0.08726646
cmd_str += "\t  	zwidth = 5.0+110*sin(zangle)
cmd_str += "\t  	global measure_width = (floor(zwidth*10))/10-9.2
cmd_str += "\t  	if slave:
cmd_str += "\t  	slave_measure_width=measure_width
cmd_str += "\t  	else:
cmd_str += "\t  	master_measure_width=measure_width
cmd_str += "\t  	end
cmd_str += "\t  	return grip_detected
cmd_str += "\t  end
cmd_str += "\t  set_tool_voltage(24)
cmd_str += "\t  set_tcp(p[0,0,0.18659,0,-0,-3.14159])
cmd_str += "\t  # end: URCap Installation Node
cmd_str += "\t  while (True):
cmd_str += "\t	$ 1 "Robot Program"
cmd_str += "\t	# begin: URCap Program Node
cmd_str += "\t	#   Source: RG - On Robot, 1.10.1, OnRobot A/S
cmd_str += "\t	#   Type: RG2
cmd_str += "\t	$ 2 "RG2(100)"
cmd_str += "\t	RG2(100,40,0.0,True,False,False)
cmd_str += "\t	# end: URCap Program Node
cmd_str += "\t  end
cmd_str += "\tend











   
	cmd_str += "\tend\n"

	print "Program({})".format(cmd_str)
	pub.publish("Program({})".format(cmd_str))

	
	print "Done"


	#rostopic pub /ur_driver/URScript std_msgs/String "data: 'movej([3.1936185359954834, -0.9609182516681116, -2.219588104878561, -1.5329936186419886, 1.5752031803131104, 1.4252062320709229], a=1.4, v=1.05, t=0, r=0)'"





if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
