from datalib import *

class DemoVariable(object):
	def __init__(self, data, derivative_data, impact_intervals):
		self.data_ = data
		self.derivative_data_ = derivative_data
		self.impact_intervals_ = impact_intervals
		self.filtered_data_ = None
		self.filtered_derivative_ = None
		self.extended_data_ = None
		self.extended_derivative_ = None
		self.extended_times_ = []
		for phase in range(self.n_phases):
			self.extended_times_.append([])
		
	@property
	def n_phases(self):
		return len(self.impact_intervals_) + 1
		
	def time_shift(self, phase):
		if len(self.impact_intervals_) > 0:
			if phase == 0:
				# Align to impact at end of phase
				return self.data_[0].time - self.data_[-1].time
			elif phase == len(self.impact_intervals_):
				# Align to impact at start of phase
				return 0
		# Align to center
		return (self.data_[0].time - self.data_[-1].time) / 2
		
	def get_starting_time(self, phase):
		return self.get_data(phase)[0].time
		
	def get_ending_time(self, phase):
		return self.get_data(phase)[-1].time
		
	def set_extended_time(self, phase, t_start, t_end):
		self.extended_times_[phase] = [t_start, t_end]
			
	def filter_data(self, data_filter):
		self.filtered_data_ = self.filter(self.data_, data_filter)
		
	def filter_derivative(self, derivative_filter):
		self.filtered_derivative_ = self.filter(self.derivative_data_, derivative_filter)
		
	def filter(self, data, data_filter):
		result = DataSet()
		
		for phase in range(self.n_phases):
			start = self.get_starting_index(phase)
			end = self.get_ending_index(phase)
			
			data_filter.reset()
			for i in range(start, end):
				filtered_datapoint, coefs = data_filter.update(data[i])
				result.append(filtered_datapoint)
			if phase < self.n_phases - 1:
				for i in range(self.impact_intervals_[phase][0], self.impact_intervals_[phase][-1] + 1):
					result.append(data[i].copy())
					
		return result
		
	def extend_data(self, extender):
		
		# Create empty lists containing the extended datasets of each phase
		self.extended_data_ = []
		self.extended_derivative_ = []
		
		# Loop through the phases
		for phase in range(self.n_phases):
			t_start, t_end = None, None
			if len(self.extended_times_[phase]) > 0:
				t_start, t_end = self.extended_times_[phase]
			extended_data, extended_derivative_data = self.extend_phase_data(extender, phase, t_start, t_end)
			self.extended_data_.append(extended_data)
			self.extended_derivative_.append(extended_derivative_data)			
			
	def extend_phase_data(self, extender, phase, t_start=None, t_end=None):
		
		if t_start is None:
			t_start = self.get_starting_time(phase)
		if t_end is None:
			t_end = self.get_ending_time(phase)
	
		# Get the data of the phase
		data = self.get_filtered_data(phase).copy()
		derivative_data = self.get_derivative_data(phase).copy()
		
		# Deleted times before and after
		deleted_before = []
		deleted_after = []
		
		# Remove None indices
		indices_to_remove = []
		for i in range(len(data)):
			if data[i].value is None or derivative_data[i].value is None:
				indices_to_remove.insert(0,i)
		for i in indices_to_remove:
			deleted_before.append(data[i].time)
			data.pop(i)
			derivative_data.pop(i)
			
		# Remove indices out of time range
		indices_to_remove = []
		for i in range(len(data)):
			time_comparison = data[i].time + self.time_shift(phase)
			if time_comparison < t_start or time_comparison > t_end:
				indices_to_remove.insert(0,i)
				if time_comparison < t_start:
					deleted_before.append(data[i].time)
				else:
					deleted_after.append(data[i].time)
					
		extend_before, extend_after = True, True
		if phase == 0:
			extend_before = False
		if len(data) == 0 or phase == self.n_phases - 1:
			extend_after = False
		
		# Extend data
		extended_data = extender.extend(data, derivative_data, extend_before=extend_before, extend_after=extend_after, extended_times_before=deleted_before, extended_times_after=deleted_after)
		extended_derivative_data = extender.extend_velocity(derivative_data, extend_before=extend_before, extend_after=extend_after, extended_times_before=deleted_before, extended_times_after=deleted_after)
		return extended_data, extended_derivative_data
		
	def get_starting_index(self, phase):
		start = 0
		if phase > 0:
			start = self.impact_intervals_[phase-1][-1] + 1
		return start
	
	def get_ending_index(self, phase):
		end = len(self.data_)
		if phase < len(self.impact_intervals_):
			end = self.impact_intervals_[phase][0]
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
		
	def get_derivative_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		return self.derivative_data_[start:end]
		
	def get_filtered_derivative(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		if self.filtered_derivative_ is not None:
			return self.filtered_derivative_[start:end]
		return self.derivative_[start:end]
	
	def get_extended_data(self, phase):
		if self.extended_data_ is not None and len(self.extended_data_) > phase:
			return self.extended_data_[phase]
		return self.get_filtered_data(phase)
		
	def get_extended_derivative(self, phase):
		if self.extended_derivative_ is not None and len(self.extended_derivative_) > phase:
			return self.extended_derivative_[phase]
		return self.get_filtered_data(phase)
		
		
