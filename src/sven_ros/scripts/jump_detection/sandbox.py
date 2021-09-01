#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *
from models import *
import config.config_create_trajectory as config
import numpy as np

position_data = []
orientation_data = []
velocity_data = []
rotational_velocity_data = []

for demo in config.demos:
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
		position[i].align_time()
		orientation[i].align_time()
		velocity[i].align_time()
		rotational_velocity[i].align_time()
		
	position_data.append(position)
	orientation_data.append(orientation)
	velocity_data.append(velocity)
	rotational_velocity_data.append(rotational_velocity)
	
position_variables = []
orientation_variables = []

for i in range(3):
	position = []
	orientation = []
	velocity = []
	rotational_velocity = []
	
	for j in range(len(config.demos)):
		position.append(position_data[j][i])
		orientation.append(orientation_data[j][i])
		velocity.append(velocity_data[j][i])
		rotational_velocity.append(rotational_velocity_data[j][i])
	
	position_variable = RobotVariable(position, velocity, config.impact_intervals)
	position_variable.filter_data(config.position_filter)
	position_variable.filter_derivative(config.velocity_filter)
	position_variable.extend_data(config.position_extender)
	position_variables.append(position_variable)
	
	orientation_variable = RobotVariable(orientation, rotational_velocity, config.impact_intervals)
	orientation_variable.filter_data(config.orientation_filter)
	orientation_variable.filter_derivative(config.rotational_velocity_filter)
	orientation_variable.extend_data(config.orientation_extender)
	orientation_variables.append(orientation_variable)
	
z = position_variables[2]
z.filter_data(config.position_filter)
z.filter_derivative(config.velocity_filter)
z.extend_data(config.position_extender)
promps = z.create_promps(config.rbf_width, config.n_rbfs_per_second)
z_pos = []
z_promps = []
for phase in range(z.n_phases):
	z_pos.append(z.demo_variables[0].get_extended_data(phase))
	mu, sigma = promps[phase].evaluate(z_pos[phase].time)
	z_promps.append(mu)
	
plt.figure()
for i in range(len(z_pos)):
	phase_data = z_pos[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	plt.plot(phase_data.time, phase_data.value,'C' + str(i) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='data')
	plt.plot(phase_data.time, z_promps[i],'C' + str(i+1) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='promp')
plt.legend()
	
plt.show()

