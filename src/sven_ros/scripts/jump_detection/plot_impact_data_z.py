#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *
from models import *
import config.config_create_trajectory as config
import config.config_evaluate_promps as config2
import numpy as np
import time as t

start_time = t.time()

position_datasets = []
orientation_datasets = []
velocity_datasets = []
rotational_velocity_datasets = []

print("Reading demonstration data")

for k in range(len(config.demos)):
	demo = config.demos[k]
	franka_reader = FrankaStateReader(demo)

	position_dataset = PositionDataSet()
	orientation_dataset = PositionDataSet()
	velocity_dataset = PositionDataSet()
	rotational_velocity_dataset = PositionDataSet()

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		position_dataset.append(PositionDataPoint(time, dp.position))
		orientation_dataset.append(PositionDataPoint(time, dp.euler_angles))
		velocity_dataset.append(PositionDataPoint(time, dp.velocity))
		rotational_velocity_dataset.append(PositionDataPoint(time, dp.rotational_velocity))
		
	position_dataset.align_time(-(position_dataset[config.impact_intervals[k][0][0]].time - position_dataset[0].time))
	orientation_dataset.align_time(-(position_dataset[config.impact_intervals[k][0][0]].time - position_dataset[0].time))
	velocity_dataset.align_time(-(position_dataset[config.impact_intervals[k][0][0]].time - position_dataset[0].time))
	rotational_velocity_dataset.align_time(-(position_dataset[config.impact_intervals[k][0][0]].time - position_dataset[0].time))
		
	position_datasets.append(position_dataset)
	orientation_datasets.append(orientation_dataset)
	velocity_datasets.append(velocity_dataset)
	rotational_velocity_datasets.append(rotational_velocity_dataset)
	
datasets_handle = RobotDataSets(position_datasets, velocity_datasets, orientation_datasets, rotational_velocity_datasets, config.impact_intervals, config.impact_detection_delay, config.impact_phase_duration)

print("--- %s seconds ---" % (t.time() - start_time))

#config2.xlim = (-0.5,0.5)

z_demos = datasets_handle.z_demos
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
plt.rcParams['xtick.labelsize'] = config2.fontsize2
plt.rcParams['ytick.labelsize'] = config2.fontsize2
for i in range(1):
	demo_data = z_demos[i]
	data0 = demo_data.get_data(0)
	data1 = demo_data.get_data(1)
	data_impact = demo_data.data_[demo_data.get_ending_index(0):demo_data.get_starting_index(1)]
	plt.plot(data0.time, data0.value,'C' + str(3*i+1) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Ante-impact phase')
	plt.plot(data_impact.time, data_impact.value,'C' + str(3*i+2) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Impact phase')
	plt.plot(data1.time, data1.value,'C' + str(3*i+3) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact phase')
	plt.xlabel('Time [s]',fontsize=config2.fontsize2)
	plt.ylabel('Position [m]',fontsize=config2.fontsize2)
	plt.title('Z position',fontsize=config2.fontsize1)
	plt.legend(fontsize=config2.fontsize2)
	if config2.xlim is not None:
		plt.xlim(config2.xlim)
		
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
plt.rcParams['xtick.labelsize'] = config2.fontsize2
plt.rcParams['ytick.labelsize'] = config2.fontsize2
for i in range(1):
	demo_data = z_demos[i]
	data0 = demo_data.get_derivative_data(0)
	data1 = demo_data.get_derivative_data(1)
	data_impact = demo_data.derivative_data_[demo_data.get_ending_index(0):demo_data.get_starting_index(1)]
	plt.plot(data0.time, data0.value,'C' + str(3*i+1) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Ante-impact phase')
	plt.plot(data_impact.time, data_impact.value,'C' + str(3*i+2) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Impact phase')
	plt.plot(data1.time, data1.value,'C' + str(3*i+3) + '-*',linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact phase')
	plt.xlabel('Time [s]',fontsize=config2.fontsize2)
	plt.ylabel('Velocity [m/s]',fontsize=config2.fontsize2)
	plt.title('Z Velocity',fontsize=config2.fontsize1)
	plt.legend(fontsize=config2.fontsize2)
	if config2.xlim is not None:
		plt.xlim(config2.xlim)

plt.show()
