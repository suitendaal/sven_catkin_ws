#!/usr/bin/python3

import numpy as np
from scipy.spatial.transform import Rotation
from datalib import *
from trajectory import *
from .position_demo_variable import *
from .orientation_demo_variable import *
from models import ProMPHandler

class RobotDataSets(object):
	def __init__(self, position_datasets, velocity_datasets, orientation_datasets, rotational_velocity_datasets, impact_intervals, normalize_orientation=True):
		self.position_datasets = position_datasets
		self.velocity_datasets = velocity_datasets
		self.orientation_datasets = orientation_datasets
		self.rotational_velocity_datasets = rotational_velocity_datasets
		self.impact_intervals = impact_intervals
		self.rotation_matrix = np.eye(3)
		if len(self.orientation_datasets) > 0 and len(self.orientation_datasets[0]) > 0:
			orientation = self.orientation_datasets[0][0].value
			rotation = Rotation.from_euler('xyz',orientation)
			self.rotation_matrix = rotation.as_matrix().T
		if normalize_orientation:
			self.normalized_orientation_datasets = self.normalize_orientation()
		else:
			self.normalized_orientation_datasets = self.orientation_datasets.copy()
		
		self.x_demos = []
		self.y_demos = []
		self.z_demos = []
		
		for i in range(len(self.position_datasets)):
			self.x_demos.append(PositionDemoVariable(self.position_datasets[i].x, self.velocity_datasets[i].x, self.impact_intervals[i]))
			self.y_demos.append(PositionDemoVariable(self.position_datasets[i].y, self.velocity_datasets[i].y, self.impact_intervals[i]))
			self.z_demos.append(PositionDemoVariable(self.position_datasets[i].z, self.velocity_datasets[i].z, self.impact_intervals[i]))
			
		self.or_x_demos = []
		self.or_y_demos = []
		self.or_z_demos = []
		
		for i in range(len(self.orientation_datasets)):
			self.or_x_demos.append(OrientationDemoVariable(self.normalized_orientation_datasets[i].x, self.impact_intervals[i]))
			self.or_y_demos.append(OrientationDemoVariable(self.normalized_orientation_datasets[i].y, self.impact_intervals[i]))
			self.or_z_demos.append(OrientationDemoVariable(self.normalized_orientation_datasets[i].z, self.impact_intervals[i]))
		
		self.set_demo_start_end()
		
	@property
	def n_phases(self):
		if len(self.impact_intervals) > 0:
			return len(self.impact_intervals[0]) + 1
		return 0
		
	@property
	def position_demos(self):
		return (self.x_demos, self.y_demos, self.z_demos)
	
	@property
	def orientation_demos(self):
		return (self.or_x_demos, self.or_y_demos, self.or_z_demos)
		
	@property
	def demos(self):
		return self.position_demos + self.orientation_demos
		
	def set_demo_start_end(self, time_of_impact_before_detecting=0):
		# Determine starting time of extended phase
		for phase in range(self.n_phases):
			t_start = self.get_starting_time(phase)
			t_end = self.get_ending_time(phase) - time_of_impact_before_detecting
			for demos in self.demos:
				for demo in demos:
					demo.set_phase_time(phase, t_start, t_end)
					
	def get_starting_time(self, phase):
		t_start = None
		for position_demo in self.x_demos:
			if t_start is None or position_demo.get_starting_time(phase) + position_demo.time_shift(phase) > t_start:
				t_start = position_demo.get_starting_time(phase) + position_demo.time_shift(phase)
		return t_start
	
	def get_ending_time(self, phase):
		t_end = None
		for position_demo in self.x_demos:
			if t_end is None or position_demo.get_ending_time(phase) + position_demo.time_shift(phase) < t_end:
				t_end = position_demo.get_ending_time(phase) + position_demo.time_shift(phase)
		return t_end
		
	def get_real_starting_time(self, phase):
		t_start = None
		for position_demo in self.x_demos:
			if t_start is None or position_demo.get_real_starting_time(phase) + position_demo.time_shift(phase) > t_start:
				t_start = position_demo.get_real_starting_time(phase) + position_demo.time_shift(phase)
		return t_start
		
	def get_real_ending_time(self, phase):
		t_end = None
		for position_demo in self.x_demos:
			if t_end is None or position_demo.get_real_ending_time(phase) + position_demo.time_shift(phase) < t_end:
				t_end = position_demo.get_real_ending_time(phase) + position_demo.time_shift(phase)
		return t_end
	
	def get_phase_starting_time(self, phase):
		t_start = None
		for position_demo in self.x_demos:
			if t_start is None or position_demo.get_phase_starting_time(phase) + position_demo.time_shift(phase) > t_start:
				t_start = position_demo.get_phase_starting_time(phase) + position_demo.time_shift(phase)
		return t_start
		
	def get_phase_ending_time(self, phase):
		t_end = None
		for position_demo in self.x_demos:
			if t_end is None or position_demo.get_phase_ending_time(phase) + position_demo.time_shift(phase) < t_end:
				t_end = position_demo.get_phase_ending_time(phase) + position_demo.time_shift(phase)
		return t_end
		
	def get_extended_starting_time(self, phase):
		t_start = None
		for position_demo in self.x_demos:
			if t_start is None or position_demo.get_extended_starting_time(phase) + position_demo.time_shift(phase) > t_start:
				t_start = position_demo.get_extended_starting_time(phase) + position_demo.time_shift(phase)
		return t_start
		
	def get_extended_ending_time(self, phase):
		t_end = None
		for position_demo in self.x_demos:
			if t_end is None or position_demo.get_extended_ending_time(phase) + position_demo.time_shift(phase) < t_end:
				t_end = position_demo.get_extended_ending_time(phase) + position_demo.time_shift(phase)
		return t_end
			
	def normalize_orientation(self):
		result = []
		for orientation_dataset in self.orientation_datasets:
			normalized_dataset = PositionDataSet()
			for orientation in orientation_dataset:
				rotation = Rotation.from_euler('xyz',orientation.value)
				new_rotation = Rotation.from_matrix(self.rotation_matrix.dot(rotation.as_matrix()))
				normalized_dataset.append(PositionDataPoint(orientation.time, new_rotation.as_euler('xyz')))
			result.append(normalized_dataset)
		return result
		
	def filter_position_data(self, filter):
		for demos in self.position_demos:
			for position_demo in demos:
				position_demo.filter_data(filter)
			
	def filter_velocity_data(self, filter):
		for demos in self.position_demos:
			for position_demo in demos:
				position_demo.filter_derivative(filter)
	
	def filter_orientation_data(self, filter):
		for demos in self.orientation_demos:
			for orientation_demo in demos:
				orientation_demo.filter_data(filter)
			
	def extend_position_data(self, extender, time_of_impact_before_detecting=0):
		self.set_demo_start_end(time_of_impact_before_detecting)
		for demos in self.position_demos:
			for position_demo in demos:
				position_demo.extend_data(extender)
			
	def extend_orientation_data(self, extender, time_of_impact_before_detecting=0):
		self.set_demo_start_end(time_of_impact_before_detecting)
		for demos in self.orientation_demos:
			for orientation_demo in demos:
				orientation_demo.extend_data(extender)
	
	def create_phase_rbfs(self, phase, width, rbfs_per_second):
		t_start = self.get_extended_starting_time(phase)
		t_end = self.get_extended_ending_time(phase)
		n_rbfs = round(rbfs_per_second * (t_end - t_start) + 1)
		rbfs = []
		for i in range(n_rbfs):
			center = (t_end - t_start) * i / (n_rbfs-1) + t_start
			rbf = RadialBasisFunction(center=center, width=width)
			rbfs.append(rbf)
		return rbfs
		
	def create_position_promps(self, rbf_width, rbfs_per_second):
		promps = PositionDataSet()
		for phase in range(self.n_phases):
			rbfs = self.create_phase_rbfs(phase, rbf_width, rbfs_per_second)
			phase_promps = []
			for demos in self.position_demos:
				phase_promps.append(self.create_phase_position_promp(phase, rbfs, demos))
			promps.append(PositionDataPoint(phase, phase_promps))
		
		# Jump intervals
		if self.n_phases > 1:
			for phase in range(self.n_phases):
				if phase < self.n_phases - 1:
					jump_intervals = []
					for demo in self.x_demos:
						jump_intervals.append(demo.get_data(phase+1)[0].time - demo.get_data(phase)[-1].time)
					for i in range(len(promps[phase].value)):
						promps[phase].value[i].next_starting_time = np.mean(jump_intervals)
				if phase > 0:
					jump_intervals = []
					for demo in self.x_demos:
						jump_intervals.append(demo.get_data(phase)[0].time - demo.get_data(phase-1)[-1].time)
					for i in range(len(promps[phase].value)):
						promps[phase].value[i].previous_ending_time = np.mean(jump_intervals)
		
		# Align promps
		for phase in range(self.n_phases):
			if phase == 0:
				time_align = -promps[phase].x.extended_starting_time
			else:
				time_align = promps[phase-1].x.get_phase_start_end()[-1] - promps[phase].x.phase_starting_time + promps[phase].x.previous_ending_time
			for i in range(len(promps[phase].value)):
				promps[phase][i].value.align_time(time_align)
			
		return promps
		
	def create_phase_position_promp(self, phase, rbfs, demos):
		t_start_phase = self.get_phase_starting_time(phase)
		t_end_phase = self.get_phase_ending_time(phase)
		t_start_real = self.get_real_starting_time(phase)
		t_end_real = self.get_real_ending_time(phase)
		t_start_extended = self.get_extended_starting_time(phase)
		t_end_extended = self.get_extended_ending_time(phase)
		
		datasets = []
		for demo in demos:
			dataset = DataSet()
			position_data = demo.get_extended_data(phase)
			velocity_data = demo.get_extended_derivative(phase)
			time_shift = demo.time_shift(phase)
			for i in range(len(position_data)):
				dataset.append(DataPoint(position_data[i].time + time_shift, [position_data[i].value, velocity_data[i].value]))
			datasets.append(dataset)
		
		promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
		promp.learn(datasets)
		mp_handler = ProMPHandler(promp, phase_starting_time=t_start_phase, phase_ending_time=t_end_phase, starting_time=t_start_real, ending_time=t_end_real, extended_starting_time=t_start_extended, extended_ending_time=t_end_extended)
		
		return mp_handler
		
	def create_orientation_promps(self, rbf_width, rbfs_per_second):
		promps = PositionDataSet()
		for phase in range(self.n_phases):
			rbfs = self.create_phase_rbfs(phase, rbf_width, rbfs_per_second)
			phase_promps = []
			for demos in self.orientation_demos:
				phase_promps.append(self.create_phase_orientation_promp(phase, rbfs, demos))
			promps.append(PositionDataPoint(phase, phase_promps))
		
		# Jump intervals
		if self.n_phases > 1:
			for phase in range(self.n_phases):
				if phase < self.n_phases - 1:
					jump_intervals = []
					for demo in self.x_demos:
						jump_intervals.append(demo.get_data(phase+1)[0].time - demo.get_data(phase)[-1].time)
					for i in range(len(promps[phase].value)):
						promps[phase].value[i].next_starting_time = np.mean(jump_intervals)
				if phase > 0:
					jump_intervals = []
					for demo in self.x_demos:
						jump_intervals.append(demo.get_data(phase)[0].time - demo.get_data(phase-1)[-1].time)
					for i in range(len(promps[phase].value)):
						promps[phase].value[i].previous_ending_time = np.mean(jump_intervals)
		
		# Align promps
		for phase in range(self.n_phases):
			if phase == 0:
				time_align = -promps[phase].x.extended_starting_time
			else:
				time_align = promps[phase-1].x.get_phase_start_end()[-1] - promps[phase].x.phase_starting_time + promps[phase].x.previous_ending_time
			for i in range(len(promps[phase].value)):
				promps[phase][i].value.align_time(time_align)
			
		return promps
		
	def create_phase_orientation_promp(self, phase, rbfs, demos):
		t_start_phase = self.get_phase_starting_time(phase)
		t_end_phase = self.get_phase_ending_time(phase)
		t_start_real = self.get_real_starting_time(phase)
		t_end_real = self.get_real_ending_time(phase)
		t_start_extended = self.get_extended_starting_time(phase)
		t_end_extended = self.get_extended_ending_time(phase)
		
		datasets = []
		for demo in demos:
			dataset = DataSet()
			position_data = demo.get_extended_data(phase)
			time_shift = demo.time_shift(phase)
			for i in range(len(position_data)):
				dataset.append(DataPoint(position_data[i].time + time_shift, position_data[i].value))
			datasets.append(dataset)
		
		promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
		promp.learn(datasets)
		mp_handler = ProMPHandler(promp, phase_starting_time=t_start_phase, phase_ending_time=t_end_phase, starting_time=t_start_real, ending_time=t_end_real, extended_starting_time=t_start_extended, extended_ending_time=t_end_extended)
		
		return mp_handler
		
