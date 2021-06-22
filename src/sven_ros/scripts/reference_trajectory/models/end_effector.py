#!/usr/bin/python3

import numpy as np
from .cartesian_data import *
from .promp_handler import *
from trajectory import *
from datalib import *

class EndEffector(object):
	"""docstring for Joint."""

	def __init__(self, n_phases, position_filter=None, velocity_estimator=None, orientation_filter=None, orientation_velocity_estimator=None, **kwargs):
		self.cartesian_data = []
		self.position_filter = position_filter
		self.velocity_estimator = velocity_estimator
		self.orientation_filter = orientation_filter
		self.orientation_velocity_estimator = orientation_velocity_estimator
		self.position_extender = kwargs.get('position_extender')
		self.orientation_extender = kwargs.get('orientation_extender')
		self.n_phases = n_phases
		self.pos_promps = []
		self.or_promps = []
		for i in range(n_phases):
			self.pos_promps.append([])
			self.or_promps.append([])
		
	def append_data(self, x, y, z, q, jump_intervals):
		self.cartesian_data.append(CartesianData(x,y,z,q,jump_intervals))
		
	def filter(self, datasets=None):
		if datasets is None:
			datasets = range(len(self.cartesian_data))
		for i in datasets:
			self.cartesian_data[i].filter(position_filter=self.position_filter, velocity_estimator=self.velocity_estimator, orientation_filter=self.orientation_filter, orientation_velocity_estimator=self.orientation_velocity_estimator)
			
	def align_time(self, dataset, phase):
		if phase == 0:
			# Align to impact
			dataset.align_time(dataset[-1].time - dataset[0].time)
		elif phase == len(self.cartesian_data[0].jump_intervals):
			# Align to start
			dataset.align_time()
		else:
			# Align to center
			dataset.align_time((dataset[-1].time - dataset[0].time) / 2)
			
	def extend_position_velocity_data(self, dataset, phase, deleted_before=[], deleted_after=[]):
		position_data = dataset.copy()
		velocity_data = dataset.copy()
		for i in range(len(dataset)):
			position_data[i].value = dataset[i].value[0]
			velocity_data[i].value = dataset[i].value[1]
		if self.position_extender is not None:
			extend_before = True
			extend_after = True
			if phase == 0:
				extend_before = False
			if len(self.cartesian_data) == 0 or phase == self.n_phases - 1:
				extend_after = False
			
			position_data = self.position_extender.extend(position_data, velocity_data, extend_before=extend_before, extend_after=extend_after, extended_times_before=deleted_before, extended_times_after=deleted_after)
			velocity_data = self.position_extender.extend_velocity(velocity_data, extend_before=extend_before, extend_after=extend_after, extended_times_before=deleted_before, extended_times_after=deleted_after)
			
			result = DataSet(timefactor=dataset.timefactor)
			for i in range(len(position_data)):
				datapoint = DataPoint(position_data[i].timestamp, [position_data[i].value, velocity_data[i].value])
				datapoint.time = position_data[i].time
				result.append(datapoint)
			return result
		return dataset.copy()
			
	# Phase is which part of the interval
	def create_promps(self, phase, rbfs, promp_type='all'):
		t_start, t_end = self.get_time_range(phase)
		if promp_type == 'position' or promp_type == 'all':
			for i in range(3):
			
				# Create ProMP
				datasets = []
				
				# Times of entire phase
				phase_starting_time = t_start
				phase_ending_time = t_end
				
				# Times of usable phase data
				starting_time = None
				ending_time = None
				
				# Times of extended phase. MP is valid for these times.
				extended_starting_time = None
				extended_ending_time = None
				
				for j in self.cartesian_data:
					if j.filtered:
					
						# Select position data
						if i == 0:
							dataset = j.get_x_filtered(phase)
						elif i == 1:
							dataset = j.get_y_filtered(phase)
						else:
							dataset = j.get_z_filtered(phase)
							
						indexes_to_pop = []
						deleted_before = []
						deleted_after = []
							
						# Select velocity data
						if self.velocity_estimator is not None:
							if i == 0:
								vel_est = j.get_x_vel(phase)
							elif i == 1:
								vel_est = j.get_y_vel(phase)
							else:
								vel_est = j.get_z_vel(phase)
								
							# Remove None indexes
							for k in range(len(dataset)):
								if vel_est[k].value is None:
									indexes_to_pop.insert(0,k)
								else:
									dataset[k].value = [dataset[k].value, vel_est[k].value]
									
						self.align_time(dataset, phase)
						
						for k in indexes_to_pop:
							deleted_before.append(dataset[k].time)
							dataset.pop(k)
								
						# Remove indexes that fall out of the time range
						indexes_to_pop = []
						for k in range(len(dataset)):
							if dataset[k].time < t_start or dataset[k].time > t_end:
								indexes_to_pop.insert(0,k)
								if dataset[k].time < t_start:
									deleted_before.append(dataset[k].time)
								else:
									deleted_after.append(dataset[k].time)
						for k in indexes_to_pop:
							dataset.pop(k)
							
						if starting_time is None or dataset[0].time > starting_time:
							starting_time = dataset[0].time
						if ending_time is None or dataset[-1].time < ending_time:
							ending_time = dataset[-1].time
								
						# Extend trajectory
						if self.velocity_estimator is not None:
							dataset = self.extend_position_velocity_data(dataset, phase, deleted_before=deleted_before, deleted_after=deleted_after)
						
						if extended_starting_time is None or dataset[0].time > extended_starting_time:
							extended_starting_time = dataset[0].time
						if extended_ending_time is None or dataset[-1].time < extended_ending_time:
							extended_ending_time = dataset[-1].time
						
						datasets.append(dataset)
				
				if self.velocity_estimator is None:		
					promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
				else:
					promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
				promp.learn(datasets)
				mp_handler = ProMPHandler(promp, phase_starting_time=phase_starting_time, phase_ending_time=phase_ending_time, starting_time=starting_time, ending_time=ending_time, extended_starting_time=extended_starting_time, extended_ending_time=extended_ending_time)
				self.pos_promps[phase].append(mp_handler)
				
		elif promp_type == 'orientation' or promp_type == 'all':
			for i in range(4):
				datasets = []
				for j in self.cartesian_data:
					if j.filtered:
						dataset = j.get_q_filtered(phase)[i].copy()
						self.align_time(dataset, phase)
						indexes_to_pop = []
						for k in range(len(dataset)):
							if dataset[k].time < t_start or dataset[k].time > t_end:
								indexes_to_pop.insert(0,k)
						for k in indexes_to_pop:
							dataset.pop(k)
						# TODO: extend orientation
						datasets.append(dataset)
				promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
				promp.learn(datasets)
				self.or_promps[phase].append(promp)	
	
	def create_basis_functions(self, phase, width, rbfs_per_second):
		
		# Get starting and ending time
		t_start, t_end = self.get_time_range(phase)
		if self.position_extender is not None:
			extend_before = True
			extend_after = True
			if phase == 0:
				extend_before = False
			if len(self.cartesian_data) == 0 or phase == len(self.cartesian_data[0].jump_intervals):
				extend_after = False
			if extend_before:
				t_start = t_start - self.position_extender.extra_time()
			if extend_after:
				t_end = t_end + self.position_extender.extra_time()
		
		# Create basis functions
		n_rbfs = round(rbfs_per_second * (t_end - t_start) + 1)
		rbfs = []
		for i in range(n_rbfs):
			center = (t_end - t_start) * i / (n_rbfs-1) + t_start
			rbf = RadialBasisFunction(center=center, width=width)
			rbfs.append(rbf)
		
		return rbfs
		
	def get_time_range(self, phase):
		t_start = None
		t_end = None
		
		datasets = []		
		for i in self.cartesian_data:
			start, end = i.get_start_end(phase)
			dataset = i.get_x(phase)
			self.align_time(dataset, phase)
			datasets.append(dataset)
		
		for i in datasets:
			if t_start is None or i[0].time > t_start:
				t_start = i[0].time
			if t_end is None or i[-1].time < t_end:
				t_end = i[-1].time
		
		return t_start, t_end
		
	def align_promp_time(self, phase):
	
		# Position promps
		for i in range(len(self.pos_promps[phase])):
			promp = self.pos_promps[phase][i]
			
			# Jumping times
			if self.n_phases > 1:
				
				# Not the last phase
				if phase >= 0 and phase < self.n_phases - 1:
					jump_times = []
					for j in self.cartesian_data:
						jump_times.append(j.x[j.jump_intervals[phase][1]].time - j.x[j.jump_intervals[phase][0]].time)
					promp.next_starting_time = np.mean(jump_times)
				
				# Not the first phase
				if phase > 0 and phase < self.n_phases:
					jump_times = []
					for j in self.cartesian_data:
						jump_times.append(j.x[j.jump_intervals[phase-1][1]].time - j.x[j.jump_intervals[phase-1][0]].time)
					promp.previous_ending_time = np.mean(jump_times)
			
			# Align time
			if phase == 0 or (phase == -1 and len(self.pos_promps == 1)):
				time_align = -promp.phase_starting_time
			else:
				time_align = self.pos_promps[phase-1][i].get_phase_start_end()[1] - promp.phase_starting_time + promp.previous_ending_time
			promp.align_time(time_align)
			
#		for i in range(len(self.or_promps[phase])):
#			promp = self.or_promps[phase][i]
#			if phase == 0 or (phase == -1 and len(self.or_promps == 1)):
#				time_align = -promp.starting_time
#			else:
#				jump_times = []
#				for j in self.cartesian_data:
#					jump_times.append(j.x[j.jump_intervals[phase-1][1]].time - j.x[i.jump_intervals[phase-1][0]].time)
#				time_align = self.or_promps[phase-1][i].get_phase_start_end()[1] + np.mean(jump_times)
#			promp.align_time(time_align)
			
			
			
		
