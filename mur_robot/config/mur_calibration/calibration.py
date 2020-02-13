#!/usr/bin/env python
#import rospy
import math
import tf
import numpy as np

if __name__ == '__main__':

	#UR_Base
	point1=[2622.5065666, 733.299246324377, -568.929158888267]
	point8=[2621.849, 733.21345051, -570.641463]
	point18=[(point1[0]+point8[0])/2,(point1[1]+point8[1])/2]
	print "point18: ",point18

	point4=[2655.88606, 645.9633165, -568.06577659]
	point9=[2655.529836947, 645.95917, -570.14716286]
	point49=[(point4[0]+point9[0])/2,(point4[1]+point9[1])/2]
	print "point49: ",point49 

	point5=[2743.51645, 679.2467, -570.784234485812]
	point6=[2743.51841793217, 679.381, -570.74254]
	point56=[(point5[0]+point6[0])/2,(point5[1]+point6[1])/2]
	print "point56: ",point56

	point2=[2709.415762, 765.987, -571.38637]
	point7=[2708.50493412945,766.519427322487,-570.715]
	point27=[(point2[0]+point7[0])/2,(point2[1]+point7[1])/2]
	print "point27: ",point27

	point_urbase=[(point18[0]+point49[0]+point56[0]+point27[0])/4,(point18[1]+point49[1]+point56[1]+point27[1])/4]
	print "point_urbase: ",point_urbase


	## M10
	point3=[2069.677148677,831.07525312,-664.515271]
	print "point3: ",point3

	point10=[2222.88689442,451.176361419235,-664.5110577]
	print "point10: ",point10

	## z-level ur_base
	point11=[2679.856, 705.8703757, -583.80512461653]
	point12=[2684.13474072263, 764.884945293818, -584.1326]
	urbase_z_level=(point11[2]+point12[2])/2

	## Floor
	point13=[1830.3235,655.4152856, -1034.89716532425]
	point14=[1738.806077, 1005.31937608384, -1035.926]
	point15=[2532.502, 1197.57748693582, -1034.35632529389]
	point16=[3333.512, 1558.70112788155, -1034.11077372694]
	point17=[1906.0290839, 60.9012, -1032.9408]

	floor_z_level=(point13[2]+point14[2]+point15[2]+point16[2]+point17[2])/5
	z_floor_urbase_diff=urbase_z_level-floor_z_level
	print "z_floor_urbase_diff: ",z_floor_urbase_diff

	#print point18
	#print point49
	#print point56
	#print point27
	
	
	v_3_urbase=[point_urbase[0]-point3[0],point_urbase[1]-point3[1]]
	print "v_3_urbase: ",v_3_urbase
	#v_0_urbase=[v_3_urbase[0]-]



	v_10_urbase=[point_urbase[0]-point10[0],point_urbase[1]-point10[1]]
	print "v_10_urbase: ",v_10_urbase



	l1=math.sqrt(v_3_urbase[0]**2 + v_3_urbase[1]**2)
	print "l1: ",l1

	l2=math.sqrt(v_10_urbase[0]**2 + v_10_urbase[1]**2)
	print "l2: ",l2

	#Distance between the two M10 Threads ... 410 - 409.6295
	l3= math.sqrt((point3[0]-point10[0])**2 + (point3[1]-point10[1])**2 )
	print "l3: ",l3


	alpha10=math.acos((l2**2 +l3**2 - l1**2)/(2*l2*l3))
	alpha3=math.acos((-l2**2 +l3**2 + l1**2)/(2*l1*l3))

	print "alpha10: ",math.degrees(alpha10)
	print "alpha3: ",math.degrees(alpha3)

	x1=l1*math.sin(alpha3) - 277.41
	y1=l1*math.cos(alpha3) -l3/2

	print x1, y1, z_floor_urbase_diff

	trans=[x1,y1,z_floor_urbase_diff]

	alpha_10_3=math.atan((point3[1]-point10[1])/(point3[0]-point10[0]))

	print "alpha_10_3: ", math.degrees(alpha_10_3)

	alpha_49_18=math.atan((point18[1]-point49[1])/(point18[0]-point49[0]))
	alpha_56_27=math.atan((point27[1]-point56[1])/(point27[0]-point56[0]))

	print math.degrees(alpha_49_18),math.degrees(alpha_56_27)
	 
	alpha_18_27=math.atan((point27[1]-point18[1])/(point27[0]-point18[0])) - math.pi/2
	alpha_49_56=math.atan((point56[1]-point49[1])/(point56[0]-point49[0])) - math.pi/2

	print math.degrees(alpha_18_27),math.degrees(alpha_49_56)

	ur_base_orientation_diff= alpha_10_3 -(alpha_49_18+alpha_56_27+alpha_18_27+alpha_49_56)/4

	print "ur_base_orientation_diff: ",math.degrees(ur_base_orientation_diff),"rad: ",ur_base_orientation_diff

	ur_base_orientation=math.pi -ur_base_orientation_diff
	print "ur_base_orientation: ",ur_base_orientation


	R_z_urbaserotation=tf.transformations.rotation_matrix(ur_base_orientation,(0,0,1))
	print "R_z_urbaserotation: ",R_z_urbaserotation
