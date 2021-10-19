#!/usr/bin/python3

from datalib import *

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
			self.extend_before(phase, trajectory_handle, result)
		if phase < trajectory_handle.n_phases - 1:
			self.extend_after(phase, trajectory_handle, result)
		
		return result
		
	def post_impact_starting_index(self, phase, trajectory_handle):
		index = trajectory_handle.trimmed_post_impact_index(phase)
		post_impact_time = trajectory_handle.trajectory_data[index].time
		while trajectory_handle.trajectory_data[index].time <= post_impact_time + self.impact_duration and index < len(trajectory_handle.trajectory_data) - 1:
			index += 1
		return index
		
	def extend_before(self, phase, trajectory_handle, trajectory):
#		t_end = 
		
	def extend_after(self, phase, trajectory_handle, trajectory):
		pass
	
