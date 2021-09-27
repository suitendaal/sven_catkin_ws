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

	print("Analyzing " + config.demos[i])

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
		
	print("Detecting jumps")
	
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
		plt.title('External force magnitude data and predictions ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim is not None and config.xlim[i] is not None:
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
		pred_diff = abs(force_ext[i] - predictions[i])
		plt.plot(pred_diff.time, pred_diff.value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Difference')
		pred_diff_jumps = pred_diff[jump_indices[i]]
		plt.plot(pred_diff_jumps.time, pred_diff_jumps.value, 'C1*', linewidth=config.linewidth, markersize=config.markersize1)

		# Bound
		plt.plot(bounds[i].time, bounds[i].value, 'C1-*', linewidth=config.linewidth, markersize=config.markersize2, label='Bound')
		
		# Adding title and labels
		plt.title('Difference between external force magnitude data and predictions ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim is not None and config.xlim[i] is not None:
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
		plt.title('End effector position ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim is not None and config.xlim[i] is not None:
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
		plt.title('End effector velocity ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim is not None and config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
			
print("Done")
	
if config.show_figs:
	plt.show()

