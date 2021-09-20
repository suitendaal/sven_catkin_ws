#!/usr/bin/python3

from datalib import *

class OrientationDemoVariable(object):
	def __init__(self, data, impact_intervals, **kwargs):
		self.data_ = data
		self.impact_intervals_ = impact_intervals
		self.extended_data_ = None
		self.impact_detection_delay = kwargs.get('impact_detection_delay',0)
		self.impact_phase_duration = kwargs.get('impact_phase_duration',0)
		
		# Real usable data for creating trajectory
		self.real_times_ = []
		
		# Times of the entire phase, including data with None values
		self.phase_times_ = []
		
		# Times included extended trajectories
		self.extended_times_ = []
		
		for phase in range(self.n_phases):
			self.real_times_.append([])
			self.phase_times_.append([])
			self.extended_times_.append([])
		
	@property
	def n_phases(self):
		return len(self.impact_intervals_) + 1
		
	def time_shift(self, phase):
		data = self.get_data(phase)
		if len(self.impact_intervals_) > 0:
			if phase == 0:
				# Align to impact at end of phase
				return -data[-1].time
			elif phase == len(self.impact_intervals_):
				# Align to impact at start of phase
				return -data[0].time
		# Align to center
		return -(data[0].time + data[-1].time) / 2
		
	def get_starting_time(self, phase):
		return self.get_data(phase)[0].time
		
	def get_ending_time(self, phase):
		return self.get_data(phase)[-1].time
		
	def set_phase_time(self, phase, t_start, t_end):
		self.phase_times_[phase] = [t_start, t_end]
		
	def set_extended_time(self, phase, t_start, t_end):
		self.extended_times_[phase] = [t_start, t_end]
		
	def get_real_starting_time(self, phase):
		if len(self.real_times_[phase]) > 0:
			return self.real_times_[phase][0] - self.time_shift(phase)
		return self.get_starting_time(phase)
		
	def get_real_ending_time(self, phase):
		if len(self.real_times_[phase]) > 0:
			return self.real_times_[phase][-1] - self.time_shift(phase)
		return self.get_ending_time(phase)
		
	def get_phase_starting_time(self, phase):
		if len(self.phase_times_[phase]) > 0:
			return self.phase_times_[phase][0] - self.time_shift(phase)
		return self.get_starting_time(phase)
		
	def get_phase_ending_time(self, phase):
		if len(self.phase_times_[phase]) > 0:
			return self.phase_times_[phase][-1] - self.time_shift(phase)
		return self.get_ending_time(phase)
		
	def get_extended_starting_time(self, phase):
		if len(self.extended_times_[phase]) > 0:
			return self.extended_times_[phase][0] - self.time_shift(phase)
		return self.get_starting_time(phase)
		
	def get_extended_ending_time(self, phase):
		if len(self.extended_times_[phase]) > 0:
			return self.extended_times_[phase][-1] - self.time_shift(phase)
		return self.get_ending_time(phase)
		
	def extend_data(self, extender):
		
		# Create empty lists containing the extended datasets of each phase
		self.extended_data_ = []
		
		# Loop through the phases
		for phase in range(self.n_phases):
			t_start, t_end = None, None
			if len(self.phase_times_[phase]) > 0:
				t_start, t_end = self.phase_times_[phase]
			extended_data = self.extend_phase_data(extender, phase, t_start, t_end)
			self.extended_data_.append(extended_data)		
			
	def extend_phase_data(self, extender, phase, t_start=None, t_end=None):
		
		if t_start is None:
			t_start = self.get_phase_starting_time(phase)
		if t_end is None:
			t_end = self.get_phase_ending_time(phase)
	
		# Get the data of the phase
		data = self.get_data(phase).copy()
		
		# Set extension settings
		extend_before, extend_after = True, True
		if phase == 0:
			extend_before = False
		if len(data) == 0 or phase == self.n_phases - 1:
			extend_after = False
		if extend_after:
			ante_impact_velocity = self.calculate_ante_impact_velocity(phase)
		if extend_before:
			post_impact_velocity = self.calculate_post_impact_velocity(phase)
		
		# Deleted times before and after
		deleted_before, deleted_after = self.remove_indices(data, t_start, t_end, phase)
				
		# Update real times
		self.real_times_[phase] = [data[0].time + self.time_shift(phase), data[-1].time + self.time_shift(phase)]	
			
		# Initialize extended data
		extended_data = data.copy()
		
		# Extend data before
		if extend_before:
			extended_data = extender.extend(extended_data, velocity=post_impact_velocity, extend_before=extend_before, extended_times_before=deleted_before)
		
		# Extend data after
		if extend_after:
			extended_data = extender.extend(extended_data, velocity=ante_impact_velocity, extend_after=extend_after, extended_times_after=deleted_after)
		
		self.extended_times_[phase] = [extended_data[0].time + self.time_shift(phase), extended_data[-1].time + self.time_shift(phase)]
		
		return extended_data
		
	def remove_indices(self, data, t_start, t_end, phase):
		deleted_times_before = []
		deleted_times_after = []
		
		# Remove indices out of time range
		deleted_times_before.extend(self.remove_before(data, t_start, phase))
		deleted_times_after.extend(self.remove_after(data, t_end, phase))
			
		return deleted_times_before, deleted_times_after
		
	def remove_before(self, data, t_start, phase):
		deleted_before = []
	
		indices_to_remove = []
		for i in range(len(data)):
			time_comparison = data[i].time + self.time_shift(phase)
			if time_comparison < t_start:
				indices_to_remove.insert(0,i)
				deleted_before.append(data[i].time)
		for i in indices_to_remove:
			data.pop(i)
			
		return deleted_before
		
	def remove_after(self, data, t_end, phase):
		deleted_after = []
	
		indices_to_remove = []
		for i in range(len(data)):
			time_comparison = data[i].time + self.time_shift(phase)
			if time_comparison > t_end:
				indices_to_remove.insert(0,i)
				deleted_after.append(data[i].time)
		for i in indices_to_remove:
			data.pop(i)
			
		return deleted_after
		
	def get_starting_index(self, phase):
		start = 0
		if phase > 0:
			start = self.get_ending_index(phase-1) + 1
			impact_time = self.data_[start-1].time
			while self.data_[start-1].time < impact_time + self.impact_phase_duration:
				start += 1
		return start
	
	def get_ending_index(self, phase):
		end = len(self.data_)
		if phase < len(self.impact_intervals_):
			end = self.impact_intervals_[phase][0]
			impact_time = self.data_[end].time
			while self.data_[end].time > impact_time - self.impact_detection_delay:
				end -= 1
		return end
		
	def get_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		return self.data_[start:end]
		
	def get_filtered_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		if self.filtered_data_ is not None:
			return self.filtered_data_[start:end]
		return self.data_[start:end]
	
	def get_extended_data(self, phase):
		if self.extended_data_ is not None and len(self.extended_data_) > phase:
			return self.extended_data_[phase]
		return self.get_filtered_data(phase)
		
	def calculate_post_impact_velocity(self, phase):
		return 0
		
	def calculate_ante_impact_velocity(self, phase):
		return 0	
		
