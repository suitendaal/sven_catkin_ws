#!/usr/bin/python3

import matplotlib.pyplot as plt
import config
from datalib import *

for i in range(config.n_joints):

	dataset = -1
	config.joints[i].detect_jumps(datasets=[dataset])

	pos_data = config.joints[i].joint_data[dataset].pos_data
	filtered_data = config.joints[i].joint_data[dataset].filtered_data
	vel_estimation = config.joints[i].joint_data[dataset].velocity_estimation
	predictions = config.joints[i].joint_data[dataset].info[0]
	bounds = config.joints[i].joint_data[dataset].info[1]
	jumping_indexes = config.joints[i].joint_data[dataset].jumping_indexes
	pos_jump = config.joints[i].joint_data[dataset].jumps()
	starting_time = config.joints[i].joint_data[dataset].starting_time()

	fontsize1 = 20
	fontsize2 = 16
	
	## Plot position data, filtered data and predictions
	fig1 = plt.figure()

	# Position data
	x0,y0 = (pos_data - filtered_data[0]).get_xy()
	plt.plot(x0,y0,'C0-',linewidth=2)

	# Filtered data
	x1,y1 = (filtered_data - filtered_data[0]).get_xy()
	plt.plot(x1,y1,'C1-',linewidth=2)

	# Predicted data
	x2,y2 = (predictions - filtered_data[0]).get_xy()
	plt.plot(x2,y2,'C2-',linewidth=2)
	
	# Jump times
	x3,y3 = (pos_jump - filtered_data[0]).get_xy()
	plt.plot(x3,y3,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Joint ' + str(i+1) + ': Position',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Position [rad]',fontsize=fontsize2)
	
	## Plot difference between predictions and bounds
	fig1 = plt.figure()
	
	# Difference between prediction and encoder data
	x4,y4 = (abs(pos_data - predictions)).get_xy()
	plt.plot(x4,y4,'C0-',linewidth=2)
	
	# Bound
	x5,y5 = bounds.get_xy()
	plt.plot(x5,y5,'C1-',linewidth=2)
	
	# Jump times
	x6,y6 = DataSet([abs(pos_data - predictions)[index] for index in jumping_indexes],timefactor=1000000).align_time(starting_time).get_xy()
	plt.plot(x6,y6,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Joint ' + str(i+1) + ': Prediction',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Position [rad]',fontsize=fontsize2)
	
plt.show()
