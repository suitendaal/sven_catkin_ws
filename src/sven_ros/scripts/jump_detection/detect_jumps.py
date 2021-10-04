#!/usr/bin/python3

import sys
import pickle
import matplotlib.pyplot as plt
import numpy as np
import config.config_jump_detection as config
from readers import *
from datalib import *

# Initialize results
force_ext = []
predictions = []
bounds = []
jump_indices = []
impact_indices = []
position = []
velocity = []
mean_errors = []

for i in range(len(config.demos)):

	print("Analyzing " + config.demos[i])

	#### Analyze the data
	
	# Initialize results
	force_ext.append(DataSet())
	predictions.append(DataSet())
	bounds.append(DataSet())
	jump_indices.append([])
	impact_indices.append([])
	position.append([])
	velocity.append([])
	for j in range(3):
		position[i].append(DataSet())
		velocity[i].append(DataSet())

	# Read data
	demo = config.demos[i]
	franka_reader = FrankaStateReader(demo)
	starting_position = None
	starting_index = None
	ending_index = None
	ending_position = franka_reader.last_datapoint().value.position
	while not franka_reader.end():
		dp = franka_reader.next_datapoint()
		time = dp.time
		franka_state = dp.value
		
		if starting_position is None:
			starting_position = franka_state.position
		elif starting_index is None:
			if franka_state.distance(starting_position) > 0.01:
				starting_index = len(force_ext[i])
		elif ending_index is None:
			if franka_state.distance(ending_position) < 0.01 and np.linalg.norm(franka_state.velocity) < 0.01:
				ending_index = len(force_ext[i]) + 1
		
		force_ext[i].append(DataPoint(time, franka_state.force_external_magnitude))
		for j in range(3):
			position[i][j].append(DataPoint(time, franka_state.position[j]))
			velocity[i][j].append(DataPoint(time, franka_state.velocity[j]))
	if ending_index is None:
		ending_index = -1
	
	# Align time
	force_ext[i].align_time()
	for j in range(3):
		position[i][j].align_time()
		velocity[i][j].align_time()
		
	print("Detecting jumps")
	
	# Detect jumps
	jump_detector = config.jump_detector
	jump_detector.reset()
	for j in range(len(force_ext[i][starting_index:ending_index])):
		jump_detected, info = jump_detector.update(force_ext[i][j+starting_index])
		if jump_detected:
			impact_indices[i].append(j+starting_index)
		if info[5]:
			jump_indices[i].append(j+starting_index)
		predictions[i].append(info[0])
		bounds[i].append(info[1])
		
	#### Output the data
	
	# Print result
	print(f"For demo {demo}, the jump indices are", jump_indices[i], "with jump times", force_ext[i][jump_indices[i]].time)
	print(f"For demo {demo}, the impact indices are", impact_indices[i], "with impact times", force_ext[i][impact_indices[i]].time)
	pred_diff = abs(force_ext[i][starting_index:ending_index] - predictions[i])
	mean_error = np.sqrt(np.mean([i**2 for i in pred_diff.value if i is not None]))
	mean_errors.append(mean_error)
	print(f'For demonstration {demo} the mean squared absolute difference between data and prediction is {mean_error}')
	print(f'{max([i for i in pred_diff.value if i is not None])}')
	
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
		force_ext_impacts = force_ext[i][impact_indices[i]]
		plt.plot(force_ext_impacts.time, force_ext_impacts.value, 'C0*', linewidth=config.linewidth, markersize=config.markersize1,label='Impacts')

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
		plt.plot(pred_diff.time, pred_diff.value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Difference')
		pred_diff_jumps = pred_diff[[index - starting_index for index in jump_indices[i]]]
		plt.plot(pred_diff_jumps.time, pred_diff_jumps.value, 'C2*', linewidth=config.linewidth, markersize=config.markersize1)
		pred_diff_impacts = pred_diff[[index - starting_index for index in impact_indices[i]]]
		plt.plot(pred_diff_jumps.time, pred_diff_jumps.value, 'C0*', linewidth=config.linewidth, markersize=config.markersize1,label='Impacts')

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
			position_jumps = position[i][j][impact_indices[i]]
			plt.plot(position_jumps.time, (position_jumps-position[i][j][0]).value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
		
		# Adding title and labels
		plt.title('End effector position ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Position [m]',fontsize=config.fontsize2)
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
			velocity_jumps = velocity[i][j][impact_indices[i]]
			plt.plot(velocity_jumps.time, velocity_jumps.value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
		
		# Adding title and labels
		plt.title('End effector velocity ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Velocity [m/s]',fontsize=config.fontsize2)
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
			
print(f'The mean mean error is {np.mean(mean_errors)} and has a minumum value of {min(mean_errors)} and a maximum value of {max(mean_errors)}')
print("Done")
	
if config.show_figs:
	plt.show()

