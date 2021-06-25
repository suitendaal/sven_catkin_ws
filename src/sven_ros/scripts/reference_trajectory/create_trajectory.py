#!/usr/bin/python3

import json
import numpy as np
import matplotlib.pyplot as plt
import config
from writers import *
from config import end_effector

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
