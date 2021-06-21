#!/usr/bin/python3

import numpy as np
from .cartesian_data import *
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
			
	def extend_position_velocity_data(self, dataset, phase):
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
			if len(self.cartesian_data) == 0 or phase == len(self.cartesian_data[0].jump_intervals):
				extend_after = False
			
			position_data = self.position_extender.extend(position_data, velocity_data, extend_before=extend_before, extend_after=extend_after)
			velocity_data = self.position_extender.extend_velocity(velocity_data, extend_before=extend_before, extend_after=extend_after)
			
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
				datasets = []
				for j in self.cartesian_data:
					if j.filtered:
					
						# Select position data
						if i == 0:
							dataset = j.get_x_filtered(phase).copy()
						elif i == 1:
							dataset = j.get_y_filtered(phase).copy()
						else:
							dataset = j.get_z_filtered(phase).copy()
							
						# Select velocity data
						if self.velocity_estimator is not None:
							if i == 0:
								vel_est = j.get_x_vel(phase).copy()
							elif i == 1:
								vel_est = j.get_y_vel(phase).copy()
							else:
								vel_est = j.get_z_vel(phase).copy()
								
							# Remove None indexes
							indexes_to_pop = []
							for k in range(len(dataset)):
								if vel_est[k].value is None:
									indexes_to_pop.insert(0,k)
								else:
									dataset[k].value = [dataset[k].value, vel_est[k].value]
							for k in indexes_to_pop:
								dataset.pop(k)
								
						# Remove indexes that fall out of the time range
						self.align_time(dataset, phase)
						indexes_to_pop = []
						for k in range(len(dataset)):
							if dataset[k].time < t_start or dataset[k].time > t_end:
								indexes_to_pop.insert(0,k)
						for k in indexes_to_pop:
								dataset.pop(k)
								
						# Extend trajectory
						if self.velocity_estimator is not None:
							dataset = self.extend_position_velocity_data(dataset, phase)
						datasets.append(dataset)
				
				if self.velocity_estimator is None:		
					promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
				else:
					promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
				promp.learn(datasets)
				self.pos_promps[phase].append(promp)
				
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
		for i in range(len(self.pos_promps[phase])):
			promp = self.pos_promps[phase][i]
			if phase == 0 or (phase == -1 and len(self.pos_promps == 1)):
				time_align = -promp.starting_time
			else:
				jump_times = []
				for j in self.cartesian_data:
					jump_times.append(j.x[j.jump_intervals[phase-1][1]].time - j.x[j.jump_intervals[phase-1][0]].time)
				time_align = self.pos_promps[phase-1][i].get_ending_time() + promp.starting_time + np.mean(jump_times)
			promp.align_time(time_align)
			
		for i in range(len(self.or_promps[phase])):
			promp = self.or_promps[phase][i]
			if phase == 0 or (phase == -1 and len(self.or_promps == 1)):
				time_align = -promp.starting_time
			else:
				jump_times = []
				for j in self.cartesian_data:
					jump_times.append(j.x[j.jump_intervals[phase-1][1]].time - j.x[i.jump_intervals[phase-1][0]].time)
				time_align = self.or_promps[phase-1][i].get_ending_time() + np.mean(jump_times)
			promp.align_time(time_align)
			
			
			
		
