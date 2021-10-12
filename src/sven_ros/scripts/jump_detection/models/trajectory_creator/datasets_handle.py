#!/usr/bin/python3

from .trajectory_handle import *
from trajectory import *
from models import *

from scipy.spatial.transform import Rotation

class DataSetsHandle(object):
	def __init__(self, position_datasets, velocity_datasets, orientation_datasets, impact_intervals):
		self.position_trajectory_handles = []
		self.orientation_trajectory_handles = []
		
		orientation = orientation_datasets[0][0].value
		rotation = Rotation.from_euler('xyz',orientation)
		self.rotation_matrix = rotation.as_matrix().T
		
		for i in range(len(position_datasets)):
			self.position_trajectory_handles.append(TrajectoryHandle([position_datasets[i], velocity_datasets[i]], impact_intervals[i]))
			self.orientation_trajectory_handles.append(TrajectoryHandle([self.normalize_orientation(orientation_datasets[i])], impact_intervals[i]))
			
	@property
	def n_phases(self):
		return self.position_trajectory_handles[0].n_phases
		
	def normalize_orientation(self, orientation_dataset):
		result = PositionDataSet()
		for orientation in orientation_dataset:
			rotation = Rotation.from_euler('xyz',orientation.value)
			new_rotation = Rotation.from_matrix(self.rotation_matrix.dot(rotation.as_matrix()))
			result.append(PositionDataPoint(orientation.time, new_rotation.as_euler('xyz')))
		return result
	
	def extend_data(self, trajectory_handles, extender):
		for trajectory_handle in trajectory_handles:
			trajectory_handle.extend_data(extender)
	
	def extend_position_data(self, extender):
		self.extend_data(self.position_trajectory_handles, extender)
		
	def extend_orientation_data(self, extender):
		self.extend_data(self.orientation_trajectory_handles, extender)
		
	def align_phase_times(self, trajectory_handles, trim=False):
		for phase in range(self.n_phases):
			if phase == 0 and self.n_phases > 1:
				# Align at end
				for trajectory_handle in trajectory_handles:
					time_shift = -trajectory_handle.get_phase_ending_time(phase)
					trajectory_handle.set_phase_time_shift(phase, time_shift)
			elif phase == self.n_phases - 1 and self.n_phases > 1:
				# Align at start
				for trajectory_handle in trajectory_handles:
					time_shift = -trajectory_handle.get_phase_starting_time(phase)
					trajectory_handle.set_phase_time_shift(phase, time_shift)
			else:
				# Align at middle
				for trajectory_handle in trajectory_handles:
					time_shift = -(trajectory_handle.get_phase_starting_time(phase) + trajectory_handle.get_phase_ending_time(phase)) / 2
					trajectory_handle.set_phase_time_shift(phase, time_shift)
		
		if trim:
			self.trim_data(trajectory_handles)
		
	def align_position_data(self, trim=False):
		self.align_phase_times(self.position_trajectory_handles, trim)
		
	def align_orientation_data(self, trim=False):
		self.align_phase_times(self.orientation_trajectory_handles, trim)
		
	def get_phase_starting_time(self, phase, trajectory_handles=None):
		if trajectory_handles is None:
			trajectory_handles = self.position_trajectory_handles
		
		t_start = None
		for trajectory_handle in trajectory_handles:
			handle_t_start = trajectory_handle.get_phase_starting_time(phase)
			if t_start is None or handle_t_start > t_start:
				t_start = handle_t_start
		return t_start
		
	def get_phase_ending_time(self, phase, trajectory_handles=None):
		if trajectory_handles is None:
			trajectory_handles = self.position_trajectory_handles
		
		t_end = None
		for trajectory_handle in trajectory_handles:
			handle_t_end = trajectory_handle.get_phase_ending_time(phase)
			if t_end is None or handle_t_end < t_end:
				t_end = handle_t_end
		return t_end
		
	def get_extended_starting_time(self, phase, trajectory_handles=None):
		if trajectory_handles is None:
			trajectory_handles = self.position_trajectory_handles
		
		t_start = None
		for trajectory_handle in trajectory_handles:
			handle_t_start = trajectory_handle.get_extended_starting_time(phase)
			if t_start is None or handle_t_start > t_start:
				t_start = handle_t_start
		return t_start
		
	def get_extended_ending_time(self, phase, trajectory_handles=None):
		if trajectory_handles is None:
			trajectory_handles = self.position_trajectory_handles
		
		t_end = None
		for trajectory_handle in trajectory_handles:
			handle_t_end = trajectory_handle.get_extended_ending_time(phase)
			if t_end is None or handle_t_end < t_end:
				t_end = handle_t_end
		return t_end
		
	def trim_data(self, trajectory_handles):
		for phase in range(self.n_phases):
			t_start = self.get_phase_starting_time(phase, trajectory_handles)
			t_end = self.get_phase_ending_time(phase, trajectory_handles)
			
			for trajectory_handle in trajectory_handles:
				trajectory_handle.trim_phase(phase, t_start, t_end)
				
	def create_promps(self, trajectory_handles, rbf_width, rbfs_per_second):
		promps = PositionDataSet()
		
		for phase in range(self.n_phases):
			promps.append(PositionDataPoint(phase, self.create_phase_promps(trajectory_handles, phase, rbf_width, rbfs_per_second)))
			
		# Impact intervals
		if self.n_phases > 1:
			for phase in range(self.n_phases):
				if phase < self.n_phases - 1:
					impact_intervals = []
					for trajectory_handle in trajectory_handles:
						impact_intervals.append(trajectory_handle.impact_duration(phase))
					for j in range(len(promps[phase].value)):
						promps[phase].value[j].next_starting_time = np.mean(impact_intervals)
				if phase > 0:
					impact_intervals = []
					for trajectory_handle in trajectory_handles:
						impact_intervals.append(trajectory_handle.impact_duration(phase-1))
					for j in range(len(promps[phase].value)):
						promps[phase].value[j].previous_ending_time = np.mean(impact_intervals)
		
		# Align promps
		for phase in range(self.n_phases):
			if phase == 0:
				time_align = -promps[phase].x.extended_starting_time
			else:
				time_align = promps[phase-1].x.get_phase_start_end()[-1] - promps[phase].x.phase_starting_time + promps[phase].x.previous_ending_time
			for j in range(len(promps[phase].value)):
				promps[phase][j].value.align_time(time_align)
		
		return promps
				
	def create_position_promps(self, rbf_width, rbfs_per_second):
		return self.create_promps(self.position_trajectory_handles, rbf_width, rbfs_per_second)
		
	def create_orientation_promps(self, rbf_width, rbfs_per_second):
		return self.create_promps(self.orientation_trajectory_handles, rbf_width, rbfs_per_second)
		
	def create_phase_promps(self, trajectory_handles, phase, rbf_width, rbfs_per_second):
		promps = []
		
		t_start = self.get_phase_starting_time(phase, trajectory_handles)
		t_end = self.get_phase_ending_time(phase, trajectory_handles)
		t_start_extended = self.get_extended_starting_time(phase, trajectory_handles)
		t_end_extended = self.get_extended_ending_time(phase, trajectory_handles)
		
		rbfs = self.create_phase_rbfs(trajectory_handles, phase, rbf_width, rbfs_per_second)
		
		datasets = []
		for trajectory_handle in trajectory_handles:
			datasets.append(trajectory_handle.get_extended_dataset(phase))
			
		for j in range(3):
			promp_datasets = []
			for dataset in datasets:
				promp_datasets.append(dataset.get_index(j))
					
			promp = ProMP(rbfs,weights_covariance=1,derivatives=len(promp_datasets[0][0].value)-1)
			promp.learn(promp_datasets)
			mp_handler = ProMPHandler(promp, phase_starting_time=t_start, phase_ending_time=t_end, extended_starting_time=t_start_extended, extended_ending_time=t_end_extended)
			
			promps.append(mp_handler)
		
		return promps
		
	def create_phase_rbfs(self, trajectory_handles, phase, rbf_width, rbfs_per_second):
		t_start = self.get_phase_starting_time(phase, trajectory_handles)
		t_end = self.get_phase_ending_time(phase, trajectory_handles)
		
		n_rbfs = round(rbfs_per_second * (t_end - t_start) + 1)
		rbfs = []
		for i in range(n_rbfs):
			center = (t_end - t_start) * i / (n_rbfs-1) + t_start
			rbf = RadialBasisFunction(center=center, width=rbf_width)
			rbfs.append(rbf)
		return rbfs
	
