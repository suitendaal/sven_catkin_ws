#!/usr/bin/python3

from readers import *
from statistics import mean
import matplotlib.pyplot as plt
from datalib import *
from models import *
import config.config_impact_detection_delay as config
import config.config_jump_detection as config_jd
import time as t

start_time = t.time()

force_ext_datasets = []
position_datasets = []
velocity_datasets = []

print("Reading demonstration data")

for demo in config.demos:
	franka_reader = FrankaStateReader(demo)

	position_dataset = PositionDataSet()
	velocity_dataset = PositionDataSet()
	force_ext_dataset = DataSet()

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		position_dataset.append(PositionDataPoint(time, dp.position))
		velocity_dataset.append(PositionDataPoint(time, dp.velocity))
		force_ext_dataset.append(DataPoint(time, dp.force_external_magnitude))
		
	position_dataset.align_time()
	velocity_dataset.align_time()
	force_ext_dataset.align_time()
		
	position_datasets.append(position_dataset)
	velocity_datasets.append(velocity_dataset)
	force_ext_datasets.append(force_ext_dataset)

print("--- %s seconds ---" % (t.time() - start_time))
print("Analyzing demos")

detection_delays = []
for i in range(2):
	detection_delays.append([])

for i in range(len(force_ext_datasets)):
	demo = config.demos[i]
	
	datas = []
	all_jump_indices = []
	all_predictions = []
	all_bounds = []
	ends = []
	
	for k in range(2):

		end = config.impact_intervals[i][0][k]+1
		start_compare = 0
		if k > 0:
			start_compare = config.impact_intervals[i][0][k-1]
		start = max(start_compare,round(end - 2*config_jd.jump_detector.max_window_length - 1))
		data = force_ext_datasets[i][start:end]
		impact_time = data[-1].time
		jump_detector = config_jd.jump_detector.copy()
		jump_detector.bounder.set_bound(None)
#		jump_detector.bounder.set_bound(config.bound[k])
		jump_detector.reset()
		
		predictions = DataSet()
		bounds = DataSet()
		jump_indices = []
		for j in range(len(data)):
			jump_detected, info = jump_detector.update(data[j])
			predictions.append(info[0])
			bounds.append(info[1])
			if jump_detected:
				jump_indices.append(j-1)
		
		if len(jump_indices) > 0:
			detection_delays[k].append(impact_time - data[jump_indices[-1]].time)
		
		ends.append(end)
		datas.append(data)
		all_predictions.append(predictions)
		all_bounds.append(bounds)
		all_jump_indices.append(jump_indices)
		
	
	## Plot external force
	if config.plot_external_force:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Predictions
		for k in range(2):
			plt.plot(all_predictions[k].time, all_predictions[k].value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Predictions')

		# External force data
		plt.plot(force_ext_datasets[i].time, force_ext_datasets[i].value, 'C0-*', linewidth=config.linewidth, markersize=config.markersize2, label='External force')
		for k in range(2):
			plt.plot(datas[k].time, datas[k].value, 'C' + str(k+1) + '-*', linewidth=config.linewidth, markersize=config.markersize2, label='External force data used')
		detected_jumps = force_ext_datasets[i][list(config.impact_intervals[i][0])]
		plt.plot(detected_jumps.time, detected_jumps.value, 'C0*', linewidth=config.linewidth, markersize=config.markersize1)
		jump_indices = [ji for jis in all_jump_indices for ji in jis]
		data_jumps = data[jump_indices]
		plt.plot(data_jumps.time, data_jumps.value, 'C1*', linewidth=config.linewidth, markersize=config.markersize1)
		
		# Adding title and labels
		plt.title('External force magnitude data and predictions ' + demo,fontsize=config.fontsize1)
		plt.xlabel('Time [s]',fontsize=config.fontsize2)
		plt.ylabel('Force [N]',fontsize=config.fontsize2)
		plt.legend(fontsize=config.fontsize2)
		if config.xlim is not None and config.xlim[i] is not None:
			plt.xlim(config.xlim[i])
		else:
			plt.xlim((data[0].time - (data[-1].time - data[0].time), data[-1].time + (data[-1].time - data[0].time)))
		
		title = fig.axes[0].get_title() + demo.split('/')[-1].split('\\')[-1]
		if config.save_figs:
			plt.savefig(config.save_figs_location + '/figures/' + title + '.png')

		if config.pickle_figs:
			pickle.dump(fig,open(config.save_figs_location + '/pickles/' + title + '.pickle','wb'))
			
		if not config.show_figs:
			plt.close()
			
	## Plot difference between data and prediction
	if config.plot_prediction_difference:
		for k in range(2):
			fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Difference between data and prediction
			pred_diff = abs(datas[k] - all_predictions[k])
			plt.plot(pred_diff.time, pred_diff.value, 'C2-*', linewidth=config.linewidth, markersize=config.markersize2, label='Difference')
			pred_diff_jumps = pred_diff[all_jump_indices[k]]
			plt.plot(pred_diff_jumps.time, pred_diff_jumps.value, 'C1*', linewidth=config.linewidth, markersize=config.markersize1)

			# Bound
			plt.plot(all_bounds[k].time, all_bounds[k].value, 'C1-*', linewidth=config.linewidth, markersize=config.markersize2, label='Bound')
			
			# Adding title and labels
			plt.title(f'Difference between external force magnitude data and predictions impact {k+1} ' + demo,fontsize=config.fontsize1)
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
			
	for k in range(2):
		for j in range(len(all_jump_indices[k])):
			all_jump_indices[k][j] = all_jump_indices[k][j] - (len(datas[k]) - 1) + (ends[k] - 1)
		print(k,all_jump_indices[k], ends[k], list(config.impact_intervals[i][0]))
			
	## Plot position data
	if config.plot_position:
		fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
		plt.rcParams['xtick.labelsize'] = config.fontsize2
		plt.rcParams['ytick.labelsize'] = config.fontsize2

		# Position
		for j in range(3):
			plt.plot(position_datasets[i].time, (position_datasets[i].get_index(j)-position_datasets[i].get_index(j)[0]).value, 'C' + str(j+1) + '-*', linewidth=config.linewidth, markersize=config.markersize2, label=config.labels[j])
			position_detected_jumps = position_datasets[i].get_index(j)[list(config.impact_intervals[i][0])]
			plt.plot(position_detected_jumps.time, (position_detected_jumps-position_datasets[i].get_index(j)[0]).value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
			jump_indices = [ji for jis in all_jump_indices for ji in jis]
			position_jumps = position_datasets[i].get_index(j)[jump_indices]
			plt.plot(position_jumps.time, (position_jumps-position_datasets[i].get_index(j)[0]).value, 'C0*', linewidth=config.linewidth, markersize=config.markersize1)
		
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

		# Velocity
		for j in range(3):
			plt.plot(velocity_datasets[i].time, velocity_datasets[i].get_index(j).value, 'C' + str(j+1) + '-*', linewidth=config.linewidth, markersize=config.markersize2, label=config.labels[j])
			velocity_detected_jumps = velocity_datasets[i].get_index(j)[list(config.impact_intervals[i][0])]
			plt.plot(velocity_detected_jumps.time, velocity_detected_jumps.value, 'C' + str(j+1) + '*', linewidth=config.linewidth, markersize=config.markersize1)
			jump_indices = [ji for jis in all_jump_indices for ji in jis]
			velocity_jumps = velocity_datasets[i].get_index(j)[jump_indices]
			plt.plot(velocity_jumps.time, velocity_jumps.value, 'C0*', linewidth=config.linewidth, markersize=config.markersize1)
		
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
		

print(f'Detection delays: {detection_delays}')
for k in range(2):
	if len(detection_delays[k]) == 0:
		detection_delays[k].append(0)
	print(f'Maximal detection delay for {k+1}st impact is {max(detection_delays[k])}, average_detection_delay is {mean(detection_delays[k])}')
print("--- %s seconds ---" % (t.time() - start_time))
print("Done")

plt.show()

