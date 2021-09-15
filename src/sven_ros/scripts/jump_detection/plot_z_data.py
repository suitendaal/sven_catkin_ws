#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_create_trajectory as config
import config.config_evaluate_promps as config2
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP

# Initialize data arrays
position_data = []
orientation_data = []
velocity_data = []
rotational_velocity_data = []

print("Reading demonstration data")

# Read data of each demo
for k in range(len(config.demos)):
	demo = config.demos[k]
	franka_reader = FrankaStateReader(demo)

	position = []
	orientation = []
	velocity = []
	rotational_velocity = []
	for i in range(3):
		position.append(DataSet())
		orientation.append(DataSet())
		velocity.append(DataSet())
		rotational_velocity.append(DataSet())

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		for j in range(3):
			position[j].append(DataPoint(time, dp.position[j]))
			orientation[j].append(DataPoint(time, dp.euler_angles[j]))
			velocity[j].append(DataPoint(time, dp.velocity[j]))
			rotational_velocity[j].append(DataPoint(time, dp.rotational_velocity[j]))
		
	for i in range(3):
		position[i].align_time(-(position[i][config.impact_intervals[k][0][0]].time - position[i][0].time))
		orientation[i].align_time(-(position[i][config.impact_intervals[k][0][0]].time - position[i][0].time))
		velocity[i].align_time(-(position[i][config.impact_intervals[k][0][0]].time - position[i][0].time))
		rotational_velocity[i].align_time(-(position[i][config.impact_intervals[k][0][0]].time - position[i][0].time))
		
	position_data.append(position)
	orientation_data.append(orientation)
	velocity_data.append(velocity)
	rotational_velocity_data.append(rotational_velocity)
	
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
for i in range(len(position_data)):
	data = position_data[i][2]
	plt.rcParams['xtick.labelsize'] = config2.fontsize2
	plt.rcParams['ytick.labelsize'] = config2.fontsize2
	plt.plot(data.time, data.value,'C' + str(i+1) + '-*',linewidth=config2.linewidth, markersize=config2.markersize2,label='Data ' + str(i+1))
	plt.legend(fontsize=config2.fontsize2)
	plt.xlabel('Time [s]',fontsize=config2.fontsize2)
	plt.ylabel('Position [m]',fontsize=config2.fontsize2)
	plt.title('Z position',fontsize=config2.fontsize1)
	if config2.xlim is not None:
		plt.xlim(config2.xlim)
		
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
for i in range(len(velocity_data)):
	data = velocity_data[i][2]
	plt.rcParams['xtick.labelsize'] = config2.fontsize2
	plt.rcParams['ytick.labelsize'] = config2.fontsize2
	plt.plot(data.time, data.value,'C' + str(i+1) + '-*',linewidth=config2.linewidth, markersize=config2.markersize2,label='Data ' + str(i+1))
	plt.legend(fontsize=config2.fontsize2)
	plt.xlabel('Time [s]',fontsize=config2.fontsize2)
	plt.ylabel('Velocity [m/s]',fontsize=config2.fontsize2)
	plt.title('Z velocity',fontsize=config2.fontsize1)
	if config2.xlim is not None:
		plt.xlim(config2.xlim)
	
plt.show()
