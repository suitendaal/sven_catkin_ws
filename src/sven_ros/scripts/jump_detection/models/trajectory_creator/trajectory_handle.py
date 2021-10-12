#!/usr/bin/python3

from datalib import *

class TrajectoryHandle(object):
	def __init__(self, trajectory_data, impact_intervals):
		self.trajectory_data = DataSet()
		self.impact_intervals = impact_intervals
		
		for i in range(len(trajectory_data[0])):
			time = trajectory_data[0][i].time
			value = []
			for j in range(len(trajectory_data)):
				value.append(trajectory_data[j][i].value)
			self.trajectory_data.append(DataPoint(time, value))
		
		self.phase_data = []
		self.phase_time_shifts = []
		self.phase_data_removed_indices = []
		for phase in range(self.n_phases):
			self.phase_data.append(DataSet())
			self.phase_time_shifts.append(0)
			self.phase_data_removed_indices.append(None)
		
	@property
	def n_phases(self):
		return len(self.impact_intervals) + 1
		
	def get_phase_starting_time(self, phase):
		return self.trajectory_data[self.post_impact_index(phase)].time + self.phase_time_shifts[phase]
	
	def get_phase_ending_time(self, phase):
		return self.trajectory_data[self.ante_impact_index(phase)].time + self.phase_time_shifts[phase]
		
	def get_extended_starting_time(self, phase):
		if len(self.phase_data[phase]) > 0:
			return self.phase_data[phase][0].time
		return self.get_phase_starting_time(phase)
		
	def get_extended_ending_time(self, phase):
		if len(self.phase_data[phase]) > 0:
			return self.phase_data[phase][-1].time
		return self.get_phase_ending_time(phase)
		
	def get_trimmed_phase_starting_time(self, phase):
		if self.phase_data_removed_indices[phase] is None:
			return self.get_phase_starting_time(phase)
		return self.trajectory_data[self.post_impact_index(phase) + self.phase_data_removed_indices[phase][0]].time + self.phase_time_shifts[phase]
		
	def get_trimmed_phase_ending_time(self, phase):
		if self.phase_data_removed_indices[phase] is None:
			return self.get_phase_ending_time(phase)
		return self.trajectory_data[self.ante_impact_index(phase) + self.phase_data_removed_indices[phase][-1]].time + self.phase_time_shifts[phase]
		
	def post_impact_index(self, phase):
		if phase > 0:
			return self.impact_intervals[phase-1][-1]
		return 0
		
	def ante_impact_index(self, phase):
		if phase < len(self.impact_intervals):
			return self.impact_intervals[phase][0]
		return len(self.trajectory_data) - 1
		
	def trimmed_post_impact_index(self, phase):
		result = self.post_impact_index(phase)
		if self.phase_data_removed_indices[phase] is not None:
			return result + self.phase_data_removed_indices[phase][0]
		return result
		
	def trimmed_ante_impact_index(self, phase):
		result = self.ante_impact_index(phase)
		if self.phase_data_removed_indices[phase] is not None:
			return result + self.phase_data_removed_indices[phase][-1]
		return result
		
	def impact_duration(self, phase):
		return self.trajectory_data[self.post_impact_index(phase+1)].time - self.trajectory_data[self.ante_impact_index(phase)].time
			
	def set_phase_time_shift(self, phase, time_shift):
		self.phase_time_shifts[phase] += time_shift
		
	def trim_phase(self, phase, t_start, t_end):
		result = []
		
		# Remove before
		starting_index = self.post_impact_index(phase)
		index = starting_index
		while index < len(self.trajectory_data) and t_start > self.trajectory_data[index].time + self.phase_time_shifts[phase]:
			index += 1
		result.append(index - starting_index)
		
		# Remove after
		starting_index = self.ante_impact_index(phase)
		index = starting_index
		while index >= 0 and t_end < self.trajectory_data[index].time + self.phase_time_shifts[phase]:
			index -= 1
		result.append(index - starting_index)
		
		self.phase_data_removed_indices[phase] = result
		
	def extend_data(self, extender):
		for phase in range(self.n_phases):
			self.phase_data[phase] = extender.extend(phase, self)
			
	def get_extended_dataset(self, phase):
		result = PositionDataSet()
		
		for datapoint in self.phase_data[phase]:
			time = datapoint.time
			value = []
			for i in range(len(datapoint.value[0])):
				tmp = []
				for j in range(len(datapoint.value)):
					tmp.append(datapoint.value[j][i])
				value.append(tmp)
			result.append(PositionDataPoint(time, value))
		return result
		
