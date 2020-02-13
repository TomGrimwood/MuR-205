#!/usr/bin/env python
#import rospy
import math
import tf
import numpy as np
import yaml
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import csv

if __name__ == '__main__':

	mean_error_all=[]
	mean_error_dev_all=[]

	diff_start_end_all=[]


	for k in range(1,11):
		print "Motion: ",k

		current_dataset=[]
		dataset=[]
		directory='/home/ros_match/mur_measurements/nullspace_motion_odom_calib/csv/'
		#, quotechar='|'
		with open(directory+'nullspace_motion_odom_calib_'+str(k)+'.csv', 'rb') as csvfile:
			datareader = csv.reader(csvfile, delimiter=',')
			for row in datareader:
				current_dataset=[float(row[0]),float(row[1]),float(row[2])]
				dataset.append(current_dataset)

		#print dataset
		#print type(dataset),type(dataset[0]),type(dataset[0][0])

		start_values_x=[]
		start_values_y=[]
		start_values_z=[]

		for i in range(0,10):
			start_values_x.append(dataset[i][0])
			start_values_y.append(dataset[i][1])
			start_values_z.append(dataset[i][2])

		start_value=[np.mean(start_values_x),np.mean(start_values_y),np.mean(start_values_z)]

		end_values_x=[]
		end_values_y=[]
		end_values_z=[]

		for i in range(len(dataset)-10,len(dataset)):
			end_values_x.append(dataset[i][0])
			end_values_y.append(dataset[i][1])
			end_values_z.append(dataset[i][2])

		end_value=[np.mean(end_values_x),np.mean(end_values_y),np.mean(end_values_z)]

		diff_start_end=math.sqrt((start_value[0]-end_value[0])**2 + (start_value[1]-end_value[1])**2 + (start_value[2]-end_value[2])**2 )

		print "start_value: ",start_value,"end_value: ",end_value,"diff_start_end: ",diff_start_end

		diff_start_end_all.append(diff_start_end)



		
		deviations=[]
		number_of_values=len(dataset)
		print number_of_values

		

		for j in range(0,len(dataset)):
			deviations.append(math.sqrt( (start_value[0] -dataset[j][0])**2 +  (start_value[1] -dataset[j][1])**2 + (start_value[2] -dataset[j][2])**2 ))
			#print j
			#print deviations

		mean_deviation=np.mean(deviations)
		std_dev_of_dev=np.std(deviations)

		print "mean_deviation: ",mean_deviation
		print "std_dev_of_dev: ",std_dev_of_dev

		protocol_dict={'number_of_values':number_of_values,'start_value':start_value,'mean_deviation':mean_deviation,'std_dev_of_dev':std_dev_of_dev}
		mean_error_all.append(mean_deviation)
		mean_error_dev_all.append(std_dev_of_dev)
		with open(directory+'protocol'+str(k)+'.json','w') as json_file:
			json.dump(protocol_dict,json_file)
	

	protocol_all_dict={'mean_error_all':mean_error_all,'mean_error_dev_all':mean_error_dev_all,'diff_start_end_all':diff_start_end_all}

	with open(directory+'protocol_all.json','w') as json_file:
			json.dump(protocol_all_dict,json_file)


	print "mean_error_all: ",mean_error_all

	print "\n\nmean_error_dev_all: ",mean_error_dev_all

	print "diff_start_end_all: ",diff_start_end_all, "Mean: ",np.mean(diff_start_end_all),"Std_dev: ",np.std(diff_start_end_all)
	


	#Create Boxplot

	plot_data=[mean_error_all,mean_error_dev_all]

	fig1, ax1 = plt.subplots()
	fig1.subplots_adjust(bottom=0.5)
	#ax1.set_title(boxplot_title)
	bp=ax1.boxplot(plot_data, vert=False,patch_artist=True)
	
	for i in [1,2]:
		x = plot_data[i-1]
		y = np.random.normal(i, 0.02, len(x))
		plt.plot(x, y, 'b.', alpha=0.6)

	#plt.show()