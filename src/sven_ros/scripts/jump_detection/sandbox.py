#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *
from models import *
import config.config_create_trajectory as config
import numpy as np

bagfile = 'data/replay4.1.bag'
franka_reader = FrankaStateReader(bagfile)

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
	
z = RobotVariable([position[2]], [velocity[2]], config.impact_intervals)
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

