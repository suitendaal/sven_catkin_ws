#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_evaluate_promps as config
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP
import time as t

def round_up(number, step_size):
	return float(Decimal(number).quantize(Decimal(str(step_size)), ROUND_UP))
	
def round_down(number, step_size):
	return float(Decimal(number).quantize(Decimal(str(step_size)), ROUND_DOWN))

start_time = t.time()

print("Reading ProMP file")

promp_reader = ProMPReader(config.promp_file)
robot_pose_evaluators = []

# Read ProMPs
for phase in range(len(promp_reader.promp_handles)):
	position_promps = []
	orientation_promps = []
	
	for i in range(3):
		position_promps.append(promp_reader.promp_handles[phase][i])
		orientation_promps.append(promp_reader.promp_handles[phase][i+3])
		
	robot_pose_evaluators.append(RobotPoseEvaluator(promp_reader.rotation_matrix, position_promps, orientation_promps))
	
print("--- %s seconds ---" % (t.time() - start_time))
print("Evaluating ProMPs")
	
# Evaluate ProMPs
datasets = []
for phase in range(len(robot_pose_evaluators)):
	datasets.append(DataSet())
	
	t_start = round_up(robot_pose_evaluators[phase].extended_starting_time, config.step_size)
	t_end = round_down(robot_pose_evaluators[phase].extended_ending_time, config.step_size)
	timerange = np.arange(t_start, t_end, config.step_size).tolist()
	
	for time in timerange:
		
		# Via points for phase
		via_points = []
		for i in range(len(config.via_points)):
			via_points.append(DataSet())
			for via_point in via_points[i]:
				if via_point.time >= t_start and via_point.time <= t_end:
					via_points[i].append(via_point)
		
		# position, velocity, orientation
		datasets[phase].append(DataPoint(time, robot_pose_evaluators[phase].evaluate(time, via_points=via_points)))
	
print("--- %s seconds ---" % (t.time() - start_time))
	
if config.write_evaluation:
	print("Writing evaluation to file")	
	
	data = dict()
	data['datasets'] = []
	for dataset in datasets:
		dataset_dict = dict()
		dataset_dict['time'] = dataset.time
		dataset_dict['position'] = dataset.get_index(0).value
		dataset_dict['velocity'] = dataset.get_index(1).value
		dataset_dict['orientation'] = dataset.get_index(2).value
		data['datasets'].append(dataset_dict)
		
	with open(config.output_file, 'w', encoding='utf-8') as f:
		json.dump(data, f, ensure_ascii=False, indent=4)
		
	print("--- %s seconds ---" % (t.time() - start_time))
	
if config.plot_figs:
	print("Plotting figures")

	for i in range(3):
		plt.figure(figsize=config.figsize,dpi=config.dpi)
		for phase in range(len(datasets)):
			pos_data = datasets[phase].get_index(0).get_index(i)
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2
			plt.plot(pos_data.time, pos_data.value,'C' + str(phase) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='Phase ' + str(phase))
		plt.legend(fontsize=config.fontsize2)
		plt.title('Position ' + config.variable_labels[i],fontsize=config.fontsize1)
		if config.xlim is not None:
			plt.xlim(config.xlim)
			
		plt.figure(figsize=config.figsize,dpi=config.dpi)
		for phase in range(len(datasets)):
			vel_data = datasets[phase].get_index(1).get_index(i)
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2
			plt.plot(vel_data.time, vel_data.value,'C' + str(phase) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='Phase ' + str(phase))
		plt.legend(fontsize=config.fontsize2)
		plt.title('Velocity ' + config.variable_labels[i],fontsize=config.fontsize1)
		if config.xlim is not None:
			plt.xlim(config.xlim)
			
		plt.figure(figsize=config.figsize,dpi=config.dpi)
		for phase in range(len(datasets)):
			or_data = datasets[phase].get_index(2).get_index(i)
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2
			plt.plot(or_data.time, or_data.value,'C' + str(phase) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='Phase ' + str(phase))
		plt.legend(fontsize=config.fontsize2)
		plt.title('Position ' + config.variable_labels[i+3],fontsize=config.fontsize1)
		if config.xlim is not None:
			plt.xlim(config.xlim)
	
	print("--- %s seconds ---" % (t.time() - start_time))
	
	plt.show()

print("Done")

