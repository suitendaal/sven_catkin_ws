#!/usr/bin/python3

from datalib import *
import numpy as np

class Extender(object):
	def __init__(self, frequency, impact_duration=0, timespan=0):
		self.frequency = frequency
		self.impact_duration = impact_duration
		self.timespan = timespan
		
	def extend(self, phase, trajectory_handle):
		starting_index = self.post_impact_starting_index(phase, trajectory_handle)
		result = trajectory_handle.trajectory_data[starting_index:trajectory_handle.trimmed_ante_impact_index(phase)].copy()
		result.align_time(trajectory_handle.trajectory_data[trajectory_handle.trimmed_post_impact_index(phase)].time + trajectory_handle.phase_time_shifts[phase])
		
		if phase > 0:
			result = self.extend_before(phase, trajectory_handle, result)
		if phase < trajectory_handle.n_phases - 1:
			result = self.extend_after(phase, trajectory_handle, result)
		
		return result
		
	def post_impact_starting_index(self, phase, trajectory_handle):
		index = trajectory_handle.trimmed_post_impact_index(phase)
		post_impact_time = trajectory_handle.trajectory_data[index].time
		while trajectory_handle.trajectory_data[index].time <= post_impact_time + self.impact_duration and index < len(trajectory_handle.trajectory_data) - 1:
			index += 1
		return index
		
	def get_times_before(self, trajectory):
		t_end = trajectory[0].time
		t_start = t_end - self.impact_duration - self.timespan
		return np.arange(t_start, t_end, 1/self.frequency)
		
	def get_times_after(self, trajectory):
		t_start = trajectory[-1].time
		t_end = t_start + self.timespan
		return np.arange(t_start, t_end, 1/self.frequency)
		
	def extend_before(self, phase, trajectory_handle, trajectory):
		times = self.get_times_before(trajectory)
		result = DataSet()
		for t in times:
			value = []
			for i in range(len(trajectory[0].value)):
				if i > 0:
					tmp = []
					for j in trajectory[0].value[i]:
						tmp.append(0)
					value.append(tmp)
				else:
					value.append(trajectory[0].value[i].copy())
			result.append(DataPoint(t, value))
		for datapoint in trajectory:
			result.append(datapoint)
		return result
		
	def extend_after(self, phase, trajectory_handle, trajectory):
		times = self.get_times_after(trajectory)
		for t in times:
			value = []
			for i in range(len(trajectory[-1].value)):
				if i > 0:
					tmp = []
					for j in trajectory[-1].value[i]:
						tmp.append(0)
					value.append(tmp)
				else:
					value.append(trajectory[-1].value[i].copy())
			trajectory.append(DataPoint(t, value))
		return trajectory
	
