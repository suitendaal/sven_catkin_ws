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
		
			fig = plt.figure(figsize=(16, 12), dpi=80)
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
			plt.plot(x3,y3,'C3*',markersize=16)
			
			# Adding title and labels
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Position',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Position [rad]',fontsize=fontsize2)
			plt.legend(['Position data','Filtered data','Predicted data','Jumps'],fontsize=fontsize2)
			plt.xlim(config.jumps_x_lim[j])
			
			if config.save_figs:
				plt.savefig(config.save_figs_location + '/' + fig.axes[0].get_title() + '.png')
				
			if not config.show_figs:
				plt.close()
		
		## Plot difference between predictions and bounds	
		if config.plot_pred:

			fig = plt.figure(figsize=(16, 12), dpi=80)
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
			plt.plot(x6,y6,'C3*',markersize=16)
			
			# Adding title and labels
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Prediction',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Position [rad]',fontsize=fontsize2)
			plt.legend(['Predictions','Bounds','Jumps'],fontsize=fontsize2)
			plt.xlim(config.jumps_x_lim[j])
			
			if config.save_figs:
				plt.savefig(config.save_figs_location + '/' + fig.axes[0].get_title() + '.png')
				
			if not config.show_figs:
				plt.close()
		
		## Plot velocity data
		if config.plot_vel:
		
			fig = plt.figure(figsize=(16, 12), dpi=80)
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
			plt.plot(x9,y9,'C3*',markersize=16)
			
			# Adding title and labels
			plt.title('Joint ' + str(i+1) + ', data ' + str(j+1) + ': Velocity',fontsize=fontsize1)
			plt.xlabel('Time [s]',fontsize=fontsize2)
			plt.ylabel('Velocity [rad/s]',fontsize=fontsize2)
			plt.legend(['Euler differentiation','Estimation','Jumps'],fontsize=fontsize2)
			plt.xlim(config.jumps_x_lim[j])
			
			if config.save_figs:
				plt.savefig(config.save_figs_location + '/' + fig.axes[0].get_title() + '.png')
				
			if not config.show_figs:
				plt.close()
		
		## Print jumping indexes
		if config.show_jumping_indexes:
			print("Joint ", i+1, ", data ", j+1, ": ", config.joints[i].joint_data[j].jumping_indexes)

if config.plot_cartesian_pos or config.plot_cartesian_vel:
	
	# Filter end effector data
	config.end_effector.filter(with_jumps=config.filter_cartesian_with_jump)
	
	for k in range(3):
	
		for j in range(len(config.demos)):
		
			if k == 0:
				pos_data = config.end_effector.cartesian_data[j].get_x()
				filtered_data = config.end_effector.cartesian_data[j].get_x_filtered()
				vel_diff = config.end_effector.cartesian_data[j].get_x_diff()
				vel_estimation = config.end_effector.cartesian_data[j].get_x_vel()
			elif k == 1:
				pos_data = config.end_effector.cartesian_data[j].get_y()
				filtered_data = config.end_effector.cartesian_data[j].get_y_filtered()
				vel_diff = config.end_effector.cartesian_data[j].get_y_diff()
				vel_estimation = config.end_effector.cartesian_data[j].get_y_vel()
			elif k == 2:
				pos_data = config.end_effector.cartesian_data[j].get_z()
				filtered_data = config.end_effector.cartesian_data[j].get_z_filtered()
				vel_diff = config.end_effector.cartesian_data[j].get_z_diff()
				vel_estimation = config.end_effector.cartesian_data[j].get_z_vel()
			
			if config.plot_cartesian_pos:
			
				fig = plt.figure(figsize=(16, 12), dpi=80)
				
				# Position data
				x0,y0 = (pos_data - filtered_data[0]).get_xy()
				plt.plot(x0,y0,'C0-',linewidth=2)

				# Filtered data
				x1,y1 = (filtered_data - filtered_data[0]).get_xy()
				plt.plot(x1,y1,'C1-',linewidth=2)
				
				# Legend
				legend = ['Position data','Filtered data']
				
				for i in range(config.n_joints):
					jumping_indexes = config.joints[i].joint_data[j].jumping_indexes
					if len(jumping_indexes) > 0:
						pos_jump = DataSet([filtered_data[index].copy() for index in jumping_indexes],timefactor=pos_data.timefactor).align_time(pos_data[jumping_indexes[0]].time)
					else:
						pos_jump = DataSet(timefactor=pos_data.timefactor)
						
					# Jump times
					x2,y2 = (pos_jump - filtered_data[0]).get_xy()
					plt.plot(x2,y2,'C' + str(i+3) + '*',markersize=16)
					
					legend.append('Joint ' + str(i+1) + ' jumps')
						
				# Adding title and labels
				if k == 0:
					plt.title('Cartesian x, data ' + str(j+1) + ': Position',fontsize=fontsize1)
				elif k == 1:
					plt.title('Cartesian y, data ' + str(j+1) + ': Position',fontsize=fontsize1)
				elif k == 2:
					plt.title('Cartesian z, data ' + str(j+1) + ': Position',fontsize=fontsize1)
				plt.xlabel('Time [s]',fontsize=fontsize2)
				plt.ylabel('Position [m]',fontsize=fontsize2)
				plt.legend(legend,fontsize=fontsize2)
				plt.xlim(config.jumps_x_lim[j])
				
				if config.save_figs:
					plt.savefig(config.save_figs_location + '/' + fig.axes[0].get_title() + '.png')
				
				if not config.show_figs:
					plt.close()
			
			if config.plot_cartesian_vel:
				fig = plt.figure(figsize=(16, 12), dpi=80)
				
				# Velocity data
				x3,y3 = vel_diff.get_xy()
				plt.plot(x3,y3,'C0-',linewidth=2)
				
				# Estimated velocity
				x4,y4 = vel_estimation.get_xy()
				plt.plot(x4,y4,'C1-',linewidth=2)
				
				# Legend
				legend = ['Euler differentiation','Estimation']
				
				for i in range(config.n_joints):
					jumping_indexes = config.joints[i].joint_data[j].jumping_indexes
				
					if len(jumping_indexes) > 0:
						vel_jump = DataSet([vel_estimation[index-1] for index in jumping_indexes],timefactor=vel_estimation.timefactor).align_time(pos_data[jumping_indexes[0]-1].time)
					else:
						vel_jump = DataSet(timefactor=vel_estimation.timefactor)
					
					# Jump times
					x5,y5 = vel_jump.get_xy()
					plt.plot(x5,y5,'C' + str(i+3) + '*',markersize=16)
					
					legend.append('Joint ' + str(i+1) + ' jumps')
				
				# Adding title and labels
				if k == 0:
					plt.title('Cartesian x, data ' + str(j+1) + ': Velocity',fontsize=fontsize1)
				elif k == 1:
					plt.title('Cartesian y, data ' + str(j+1) + ': Velocity',fontsize=fontsize1)
				elif k == 2:
					plt.title('Cartesian z, data ' + str(j+1) + ': Velocity',fontsize=fontsize1)
				plt.xlabel('Time [s]',fontsize=fontsize2)
				plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
				plt.legend(legend,fontsize=fontsize2)
				plt.xlim(config.jumps_x_lim[j])
				
				if config.save_figs:
					plt.savefig(config.save_figs_location + '/' + fig.axes[0].get_title() + '.png')
				
				if not config.show_figs:
					plt.close()
			
		
if config.show_figs:
	plt.show()
