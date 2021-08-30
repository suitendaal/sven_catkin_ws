#!/usr/bin/python3

import sys
#import json
#import numpy as np
import matplotlib.pyplot as plt
import config.config_create_trajectory as config
#from readers import *
#from datalib import *
from writers import *
from models import *

robot = Robot(config)

sys.exit()


# Initialize filtered data
position_phases = []
orientation_phases = []
velocity_phases = []
rotational_velocity_phases = []
position_filtered_phases = []
orientation_filtered_phases = []
velocity_filtered_phases = []
rotational_velocity_filtered_phases = []

for i in range(len(config.demos)):
	
	position_phases.append([])
	orientation_phases.append([])
	velocity_phases.append([])
	rotational_velocity_phases.append([])
	position_filtered_phases.append([])
	orientation_filtered_phases.append([])
	velocity_filtered_phases.append([])
	rotational_velocity_filtered_phases.append([])
	
	# Initialize data
	position_data = []
	orientation_data = []
	velocity_data = []
	rotational_velocity_data = []
	
	for j in range(3):
		position_data.append(DataSet())
		orientation_data.append(DataSet())
		velocity_data.append(DataSet())
		rotational_velocity_data.append(DataSet())
	
	# Read data
	demo = config.demos[i]
	franka_reader = FrankaStateReader(demo)
	while not franka_reader.end():
		dp = franka_reader.next_datapoint()
		time = dp.time
		franka_state = dp.value
		
		for j in range(3):
			position_data[j].append(DataPoint(time, franka_state.position[j]))
			orientation_data[j].append(DataPoint(time, franka_state.euler_angles[j]))
			velocity_data[j].append(DataPoint(time, franka_state.velocity[j]))
			rotational_velocity_data[j].append(DataPoint(time, franka_state.rotational_velocity[j]))

	# Loop through phases
	for j in range(len(config.impact_intervals[i]) + 1):
		
		# Initialize data of phase
		start = 0
		if j > 0:
			start = config.impact_intervals[i][j-1][-1] + 1
		end = len(position_data[0])
		if j < len(config.impact_intervals[i]):
			end = config.impact_intervals[i][j][0]
		
		position_phase = []
		orientation_phase = []
		velocity_phase = []
		rotational_velocity_phase = []
		position_filtered_phase = []
		orientation_filtered_phase = []
		velocity_filtered_phase = []
		rotational_velocity_filtered_phase = []
	
		for k in range(3):
			position_phase.append(position_data[k][start:end])
			orientation_phase.append(orientation_data[k][start:end])
			velocity_phase.append(velocity_data[k][start:end])
			rotational_velocity_phase.append(rotational_velocity_data[k][start:end])
			
		for k in range(3):
			position_filtered_phase.append(DataSet())
			orientation_filtered_phase.append(DataSet())
			velocity_filtered_phase.append(DataSet())
			rotational_velocity_filtered_phase.append(DataSet())
			
		## Filter data
		
		# Position data
		for k in range(3):
			data = position_phase[k]
			data_filter = config.position_filter
			data_filter.reset()
			for l in range(len(data)):
				filtered_data, coefs = data_filter.update(data[l])
				position_filtered_phase[k].append(filtered_data)
				
		# Orientation data
		for k in range(3):
			data = orientation_phase[k]
			data_filter = config.orientation_filter
			data_filter.reset()
			for l in range(len(data)):
				filtered_data, coefs = data_filter.update(data[l])
				orientation_filtered_phase[k].append(filtered_data)
				
		# Velocity data
		for k in range(3):
			data = velocity_phase[k]
			data_filter = config.velocity_filter
			data_filter.reset()
			for l in range(len(data)):
				filtered_data, coefs = data_filter.update(data[l])
				velocity_filtered_phase[k].append(filtered_data)
				
		# Rotational velocity data
		for k in range(3):
			data = rotational_velocity_phase[k]
			data_filter = config.rotational_velocity_filter
			data_filter.reset()
			for l in range(len(data)):
				filtered_data, coefs = data_filter.update(data[l])
				rotational_velocity_filtered_phase[k].append(filtered_data)
				
		





sys.exit(0)

end_effector.filter()

for phase in range(config.n_phases):

	# Create promps
	rbfs = end_effector.create_basis_functions(phase=phase, width=config.rbf_width, rbfs_per_second=config.n_rbfs_per_second)
	end_effector.create_promps(phase=phase, rbfs=rbfs, promp_type='all')
	end_effector.align_promp_time(phase)
	
for k in range(3):
	
	# Create position plots
	if config.plot_pos_data or config.plot_pos_mp:
	
		fig = plt.figure(figsize=(16, 12), dpi=80)
		
		fontsize1 = 20
		fontsize2 = 16
	
		for phase in range(config.n_phases):
		
			if config.plot_pos_data:
		
				phase_start,phase_end = end_effector.pos_promps[phase][k].get_phase_start_end()

				# Filtered position
				for i in range(len(end_effector.cartesian_data)):
					if k == 0:
						pos_data = end_effector.cartesian_data[i].get_x_filtered(phase)
					elif k == 1:
						pos_data = end_effector.cartesian_data[i].get_y_filtered(phase)
					elif k == 2:
						pos_data = end_effector.cartesian_data[i].get_z_filtered(phase)
					
					if phase == 0:
						pos_data.align_time(phase_end - pos_data[-1].time)
					elif phase == config.n_phases - 1:
						pos_data.align_time(phase_start)
					else:
						pos_data.align_time(phase_start + ((phase_end - phase_start) - (pos_data[-1].time - pos_data[0].time)) / 2)
					x,y = pos_data.get_xy()
					plt.plot(x,y,'C' + str(i+1) + '-',linewidth=2,label='data ' + str(i+1))
		
			# Plot promps
			if config.plot_pos_mp:
				# Evaluate promps
				t_start, t_end = end_effector.pos_promps[phase][k].get_extended_start_end()
				t = np.arange(t_start,t_end, 0.01).tolist()
				mu,sigma = end_effector.pos_promps[phase][k].evaluate(t, derivative=0)
				plt.plot(t,mu,'C0-',linewidth=2,label='ProMP')

		# Adding title and labels
		if k == 0:
			plt.title('Cartesian x: Position',fontsize=fontsize1)
		elif k == 1:
			plt.title('Cartesian y: Position',fontsize=fontsize1)
		elif k == 2:
			plt.title('Cartesian z: Position',fontsize=fontsize1)
		plt.xlabel('Time [s]',fontsize=fontsize2)
		plt.ylabel('Position [rad]',fontsize=fontsize2)
		handles, labels = plt.gca().get_legend_handles_labels()
		by_label = dict(zip(labels, handles))
		plt.legend(by_label.values(), by_label.keys())
		
		if config.save_trajectory_figs:
			plt.savefig(config.save_trajectory_figs_location + '/' + fig.axes[0].get_title() + '.png')
			
		if not config.show_trajectory_figs:
			plt.close()
		
	# Create position plots
	if config.plot_vel_data or config.plot_vel_mp:
	
		fig = plt.figure(figsize=(16, 12), dpi=80)
		
		fontsize1 = 20
		fontsize2 = 16
	
		for phase in range(config.n_phases):
		
			if config.plot_vel_data:
		
				phase_start,phase_end = end_effector.pos_promps[phase][k].get_phase_start_end()

				# Filtered position
				for i in range(len(end_effector.cartesian_data)):
					if k == 0:
						vel_data = end_effector.cartesian_data[i].get_x_vel(phase)
					elif k == 1:
						vel_data = end_effector.cartesian_data[i].get_y_vel(phase)
					elif k == 2:
						vel_data = end_effector.cartesian_data[i].get_z_vel(phase)
					
					if phase == 0:
						vel_data.align_time(phase_end - vel_data[-1].time)
					elif phase == config.n_phases - 1:
						vel_data.align_time(phase_start)
					else:
						vel_data.align_time(phase_start + ((phase_end - phase_start) - (vel_data[-1].time - vel_data[0].time)) / 2)
					x,y = vel_data.get_xy()
					plt.plot(x,y,'C' + str(i+1) + '-',linewidth=2,label='data ' + str(i+1))
		
			# Plot promps
			if config.plot_vel_mp:
				# Evaluate promps
				t_start, t_end = end_effector.pos_promps[phase][k].get_extended_start_end()
				t = np.arange(t_start,t_end, 0.01).tolist()
				mu,sigma = end_effector.pos_promps[phase][k].evaluate(t, derivative=1)
				plt.plot(t,mu,'C0-',linewidth=2,label='ProMP')

		# Adding title and labels
		if k == 0:
			plt.title('Cartesian x: Velocity',fontsize=fontsize1)
		elif k == 1:
			plt.title('Cartesian y: Velocity',fontsize=fontsize1)
		elif k == 2:
			plt.title('Cartesian z: Velocity',fontsize=fontsize1)
		plt.xlabel('Time [s]',fontsize=fontsize2)
		plt.ylabel('Velocity [rad/s]',fontsize=fontsize2)
		handles, labels = plt.gca().get_legend_handles_labels()
		by_label = dict(zip(labels, handles))
		plt.legend(by_label.values(), by_label.keys())
		
		if config.save_trajectory_figs:
			plt.savefig(config.save_trajectory_figs_location + '/' + fig.axes[0].get_title() + '.png')
			
		if not config.show_trajectory_figs:
			plt.close()

plt.show()

# Write promps
if config.write_mps:
	write_promps(config.output_file, end_effector.to_dict())
	
