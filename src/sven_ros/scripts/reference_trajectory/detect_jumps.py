#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
import config
from datalib import *

for i in range(config.n_joints):

	config.joints[i].detect_jumps()
	
	for j in range(len(config.joints[i].joint_data)):
		pos_data = config.joints[i].joint_data[j].pos_data
		filtered_data = config.joints[i].joint_data[j].filtered_data
		vel_estimation = config.joints[i].joint_data[j].velocity_estimation
		predictions = config.joints[i].joint_data[j].info[0]
		bounds = config.joints[i].joint_data[j].info[1]
		jumping_indexes = config.joints[i].joint_data[j].jumping_indexes
		pos_jump = config.joints[i].joint_data[j].jumps()
		starting_time = config.joints[i].joint_data[j].starting_time()
		starting_time2 = config.joints[i].joint_data[j].starting_time_before()

		fontsize1 = 20
		fontsize2 = 16
		
		## Plot position data, filtered data and predictions
		if config.plot_pos:
		
			fig0 = plt.figure()
			plt.rcParams['xtick.labelsize']=fontsize2
			plt.rcParams['ytick.labelsize']=fontsize2

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
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Position',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Position [rad]',fontsize=fontsize2)
			plt.legend(['Position data','Filtered data','Predicted data','Jumps'],fontsize=fontsize2)
		
		## Plot difference between predictions and bounds	
		if config.plot_pred:

			fig1 = plt.figure()
			plt.rcParams['xtick.labelsize']=fontsize2
			plt.rcParams['ytick.labelsize']=fontsize2
			
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
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Prediction',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Position [rad]',fontsize=fontsize2)
			plt.legend(['Predictions','Bounds','Jumps'],fontsize=fontsize2)
		
		## Plot velocity data
		if config.plot_vel:
		
			fig3 = plt.figure()
			plt.rcParams['xtick.labelsize']=fontsize2
			plt.rcParams['ytick.labelsize']=fontsize2
			
			# Velocity data
			x7,y7 = pos_data.diff().get_xy()
			plt.plot(x7,y7,'C0-',linewidth=2)
			
			# Estimated velocity
			x8,y8 = vel_estimation.get_xy()
			plt.plot(x8,y8,'C1-',linewidth=2)
			
			# Jumping times
			x9,y9 = DataSet([vel_estimation[index-1] for index in jumping_indexes],timefactor=1000000).align_time(starting_time2).get_xy()
			plt.plot(x9,y9,'C3*',markersize=10)
			
			# Adding title and labels
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Velocity',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Velocity [rad/s]',fontsize=fontsize2)
			plt.legend(['Euler differentiation','Estimation','Jumps'],fontsize=fontsize2)
		
		## Print jumping indexes
		if config.show_jumping_indexes:
			print("Joint ", i+1, ", data ", j+1, ": ", config.joints[i].joint_data[j].jumping_indexes)
	
	
plt.show()
