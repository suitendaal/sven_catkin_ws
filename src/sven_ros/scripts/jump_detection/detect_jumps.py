#!/usr/bin/python3

import sys
import pickle
import matplotlib.pyplot as plt
import config.config_jump_detection as config
from readers import *
from datalib import *

# Initialize results
force_ext = []
predictions = []
bounds = []
jump_indices = []
position = []
velocity = []

for i in range(len(config.demos)):

	#### Analyze the data
	
	# Initialize results
	force_ext.append(DataSet())
	predictions.append(DataSet())
	bounds.append(DataSet())
	jump_indices.append([])
	position.append([])
	velocity.append([])
	for j in range(3):
		position[i].append(DataSet())
		velocity[i].append(DataSet())

	# Read data
	demo = config.demos[i]
	franka_reader = FrankaStateReader(demo)
	while not franka_reader.end():
		dp = franka_reader.next_datapoint()
		time = dp.time
		franka_state = dp.value
		
		force_ext[i].append(DataPoint(time, franka_state.force_external_magnitude))
		for j in range(3):
			position[i][j].append(DataPoint(time, franka_state.position[j]))
			velocity[i][j].append(DataPoint(time, franka_state.velocity[j]))
	
	# Align time
	force_ext[i].align_time()
	for j in range(3):
		position[i][j].align_time()
		velocity[i][j].align_time()
	
	# Detect jumps
	jump_detector = config.jump_detector
	jump_detector.reset()
	for j in range(len(force_ext[i])):
		jump_detected, info = jump_detector.update(force_ext[i][j])
		if jump_detected:
			jump_indices[i].append(j)
		predictions[i].append(info[0])
		bounds[i].append(info[1])
		
	#### Output the data
	
	# Print result
	print(f"For demo {demo}, the jump indices are", jump_indices[i], "with jump times", force_ext[i][jump_indices[i]].time)
	
	### Plot figures
	
	## Plot external force
	if config.plot_external_force:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Predictions
		plt.plot(predictions[i].time, predictions[i].value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Predictions')
		force_ext_jumps = force_ext[i][jump_indices[i]]
		plt.plot(force_ext_jumps.time, force_ext_jumps.value, 'C1*', linewidth=config.linewidth, markersize=config.markersize1)

		# External force data
		plt.plot(force_ext[i].time, force_ext[i].value, 'C1-*', linewidth=config.linewidth, markersize=config.markersize2, label='External force')
		
		# Adding title and labels
		plt.title('External force magnitude data and predictions',fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
			
	## Plot difference between data and prediction
	if config.plot_prediction_difference:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Difference between data and prediction
		pred_diff = force_ext[i] - predictions[i]
		plt.plot(pred_diff.time, pred_diff.value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Difference')
		pred_diff_jumps = pred_diff[jump_indices[i]]
		plt.plot(pred_diff_jumps.time, pred_diff_jumps.value, 'C1*', linewidth=config.linewidth, markersize=config.markersize1)

		# Bound
		plt.plot(bounds[i].time, bounds[i].value, 'C1-*', linewidth=config.linewidth, markersize=config.markersize2, label='Bound')
		
		# Adding title and labels
		plt.title('Difference between external force magnitude data and predictions',fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
			
	## Plot position data
	if config.plot_position:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Position
		for j in range(3):
			plt.plot(position[i][j].time, (position[i][j]-position[i][j][0]).value, 'C' + str(j+1) + '-*', linewidth=config.linewidth, markersize=config.markersize2, label=config.labels[j])
			position_jumps = position[i][j][jump_indices[i]]
			plt.plot(position_jumps.time, (position_jumps-position[i][j][0]).value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
		
		# Adding title and labels
		plt.title('End effector position',fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
			
	## Plot velocity data
	if config.plot_velocity:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Position
		for j in range(3):
			plt.plot(velocity[i][j].time, velocity[i][j].value, 'C' + str(j+1) + '-*', linewidth=config.linewidth, markersize=config.markersize2, label=config.labels[j])
			velocity_jumps = velocity[i][j][jump_indices[i]]
			plt.plot(velocity_jumps.time, velocity_jumps.value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
		
		# Adding title and labels
		plt.title('End effector velocity',fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
	
if config.show_figs:
	plt.show()
	
sys.exit(0)
		
	
	

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
			plt.plot(x0,y0,'C0-*',linewidth=2,markersize=8)

			# Filtered data
			x1,y1 = (filtered_data - filtered_data[0]).get_xy()
			plt.plot(x1,y1,'C1-*',linewidth=2,markersize=8)

			# Predicted data
			x2,y2 = (predictions - filtered_data[0]).get_xy()
			plt.plot(x2,y2,'C2-*',linewidth=2,markersize=8)
			
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

			if config.pickle_figs:
				pickle.dump(fig,open(config.save_figs_location + '/pickle/' + fig.axes[0].get_title() + '.pickle','wb'))
				
			if not config.show_figs:
				plt.close()
		
		## Plot difference between predictions and bounds	
		if config.plot_pred:

			fig = plt.figure(figsize=(16, 12), dpi=80)
			plt.rcParams['xtick.labelsize']=fontsize2
			plt.rcParams['ytick.labelsize']=fontsize2
			
			# Difference between prediction and encoder data
			x4,y4 = (abs(pos_data - predictions)).get_xy()
			plt.plot(x4,y4,'C0-*',linewidth=2,markersize=8)
			
			# Bound
			x5,y5 = bounds.get_xy()
			plt.plot(x5,y5,'C1-*',linewidth=2,markersize=8)
			
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
				
			if config.pickle_figs:
				pickle.dump(fig,open(config.save_figs_location + '/pickle/' + fig.axes[0].get_title() + '.pickle','wb'))
				
			if not config.show_figs:
				plt.close()
		
		## Plot velocity data
		if config.plot_vel:
		
			fig = plt.figure(figsize=(16, 12), dpi=80)
			plt.rcParams['xtick.labelsize']=fontsize2
			plt.rcParams['ytick.labelsize']=fontsize2
			
			# Velocity data
			x7,y7 = pos_data.diff().get_xy()
			plt.plot(x7,y7,'C0-*',linewidth=2,markersize=8)
			
			# Estimated velocity
			x8,y8 = vel_estimation.get_xy()
			plt.plot(x8,y8,'C1-*',linewidth=2,markersize=8)
			
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
				
			if config.pickle_figs:
				pickle.dump(fig,open(config.save_figs_location + '/pickle/' + fig.axes[0].get_title() + '.pickle','wb'))
				
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
				plt.plot(x0,y0,'C0-*',linewidth=2,markersize=8)

				# Filtered data
				x1,y1 = (filtered_data - filtered_data[0]).get_xy()
				plt.plot(x1,y1,'C1-*',linewidth=2,markersize=8)
				
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
					
				if config.pickle_figs:
					pickle.dump(fig,open(config.save_figs_location + '/pickle/' + fig.axes[0].get_title() + '.pickle','wb'))
				
				if not config.show_figs:
					plt.close()
			
			if config.plot_cartesian_vel:
				fig = plt.figure(figsize=(16, 12), dpi=80)
				
				# Velocity data
				x3,y3 = vel_diff.get_xy()
				plt.plot(x3,y3,'C0-*',linewidth=2,markersize=8)
				
				# Estimated velocity
				x4,y4 = vel_estimation.get_xy()
				plt.plot(x4,y4,'C1-*',linewidth=2,markersize=8)
				
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
					
				if config.pickle_figs:
					pickle.dump(fig,open(config.save_figs_location + '/pickle/' + fig.axes[0].get_title() + '.pickle','wb'))
				
				if not config.show_figs:
					plt.close()
			
		
if config.show_figs:
	plt.show()
