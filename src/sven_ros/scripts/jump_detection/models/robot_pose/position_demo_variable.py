#!/usr/bin/python3

import math
import scipy.optimize as optimization
import numpy as np
from .orientation_demo_variable import *
import matplotlib.pyplot as plt

class PositionDemoVariable(OrientationDemoVariable):
	def __init__(self, data, derivative_data, impact_intervals):
		super(PositionDemoVariable, self).__init__(data, impact_intervals)
		self.derivative_data_ = derivative_data
		self.filtered_derivative_ = None
		self.extended_derivative_ = None
		
	def filter_derivative(self, derivative_filter):
		if derivative_filter is not None:
			self.filtered_derivative_ = self.filter(self.derivative_data_, derivative_filter)
		
	def extend_data(self, extender):
		
		# Create empty lists containing the extended datasets of each phase
		self.extended_data_ = []
		self.extended_derivative_ = []
		
		# Loop through the phases
		for phase in range(self.n_phases):
			t_start, t_end = None, None
			if len(self.phase_times_[phase]) > 0:
				t_start, t_end = self.phase_times_[phase]
			extended_data, extended_derivative_data = self.extend_phase_data(extender, phase, t_start, t_end)
			self.extended_data_.append(extended_data)
			self.extended_derivative_.append(extended_derivative_data)			
			
	def extend_phase_data(self, extender, phase, t_start=None, t_end=None):
		
		if t_start is None:
			t_start = self.get_phase_starting_time(phase)
		if t_end is None:
			t_end = self.get_phase_ending_time(phase)
	
		# Get the data of the phase
		data = self.get_filtered_data(phase).copy()
		derivative_data = self.get_filtered_derivative(phase).copy()
		
		# Deleted times before and after
		deleted_before, deleted_after = self.remove_indices(data, derivative_data, t_start, t_end, phase)
				
		# Update real times
		self.real_times_[phase] = [data[0].time + self.time_shift(phase), data[-1].time + self.time_shift(phase)]	
		
		extend_before, extend_after = True, True
		if phase == 0:
			extend_before = False
		if len(data) == 0 or phase == self.n_phases - 1:
			extend_after = False
			
		# Initialize extended data
		extended_data = data.copy()
		extended_derivative_data = derivative_data.copy()
		
		# Extend data before
		extension_velocity = 0
		if extend_before:
			extension_velocity = self.calculate_post_impact_velocity(phase, data, derivative_data)
		extended_data = extender.extend(extended_data, velocity=extension_velocity, extend_before=extend_before, extended_times_before=deleted_before)
		extended_derivative_data = extender.extend_velocity(extended_derivative_data, velocity=extension_velocity, extend_before=extend_before, extended_times_before=deleted_before)
		
		# Extend data after
		extension_velocity = 0
		if extend_after:
			extension_velocity = self.calculate_ante_impact_velocity(phase, data, derivative_data)
		extended_data = extender.extend(extended_data, velocity=extension_velocity, extend_after=extend_after, extended_times_after=deleted_after)
		extended_derivative_data = extender.extend_velocity(extended_derivative_data, velocity=extension_velocity, extend_after=extend_after, extended_times_after=deleted_after)
		
		self.extended_times_[phase] = [extended_data[0].time + self.time_shift(phase), extended_data[-1].time + self.time_shift(phase)]
		
		return extended_data, extended_derivative_data
		
	def remove_indices(self, data, derivative_data, t_start, t_end, phase):
		deleted_times_before = []
		deleted_times_after = []
		
		# Remove None indices
		deleted_times_before.extend(self.remove_none_indices(data, derivative_data))
			
		# Remove indices out of time range
		deleted_times_before.extend(self.remove_before(data, derivative_data, t_start, phase))
		deleted_times_after.extend(self.remove_after(data, derivative_data, t_end, phase))
			
		return deleted_times_before, deleted_times_after
		
	def remove_none_indices(self, data, derivative_data):
		deleted_before = []
		
		indices_to_remove = []
		for i in range(len(data)):
			if data[i].value is None or derivative_data[i].value is None:
				indices_to_remove.insert(0,i)
		for i in indices_to_remove:
			deleted_before.append(data[i].time)
			data.pop(i)
			derivative_data.pop(i)
		
		return deleted_before
		
	def remove_before(self, data, derivative_data, t_start, phase):
		deleted_before = []
	
		indices_to_remove = []
		for i in range(len(data)):
			time_comparison = data[i].time + self.time_shift(phase)
			if time_comparison < t_start:
				indices_to_remove.insert(0,i)
				deleted_before.append(data[i].time)
		for i in indices_to_remove:
			data.pop(i)
			derivative_data.pop(i)
			
		return deleted_before
		
	def remove_after(self, data, derivative_data, t_end, phase):
		deleted_after = []
	
		indices_to_remove = []
		for i in range(len(data)):
			time_comparison = data[i].time + self.time_shift(phase)
			if time_comparison > t_end:
				indices_to_remove.insert(0,i)
				deleted_after.append(data[i].time)
		for i in indices_to_remove:
			data.pop(i)
			derivative_data.pop(i)
			
		return deleted_after
		
	def get_derivative_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		return self.derivative_data_[start:end]
		
	def get_filtered_derivative(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		if self.filtered_derivative_ is not None:
			return self.filtered_derivative_[start:end]
		return self.derivative_data_[start:end]
	
	def get_extended_derivative(self, phase):
		if self.extended_derivative_ is not None and len(self.extended_derivative_) > phase:
			return self.extended_derivative_[phase]
		return self.get_filtered_data(phase)
		
	def calculate_post_impact_velocity(self, phase, data, derivative_data):
		if phase == 0:
			return 0
		ante_impact_velocity = self.get_extended_derivative(phase)[-1].value
		print("hoi",ante_impact_velocity)
		t_start = derivative_data[0].time
		t_end = t_start + 0.200
		data_to_analyze = DataSet()
		for datapoint in derivative_data:
			if datapoint.time < t_end:
				data_to_analyze.append(datapoint.copy())
		data_to_analyze.align_time()
		func = lambda t, a, A, gamma, omega, phi : fitting_func(t, a, A, gamma, omega, phi, ante_impact_velocity)
		
#		x0 = [0,0,0,0,0]
		popt, pcov = optimization.curve_fit(func, np.array(data_to_analyze.time), np.array(data_to_analyze.value))
		
		print(func([0], popt[0], popt[1], 0, 0, popt[4])[0], data_to_analyze[-1])
		
		values = func(np.array(data_to_analyze.time), popt[0], popt[1], popt[2], popt[3], popt[4])
#		for i in range(len(values)):
#			values[i] -= math.cos(popt[4])
		print(math.cos(popt[4]))
#		plt.plot(np.array(data_to_analyze.time), values)
#		plt.plot(np.array(data_to_analyze.time), np.array(data_to_analyze.value))
		plt.show()
		
		return func([0], popt[0], popt[1], 0, 0, popt[4])[0]
		
	def calculate_ante_impact_velocity(self, phase, data, derivative_data):
		return derivative_data[-1].value
		
def fitting_func(t, a, A, gamma, omega, phi, v_min):
	result = []
	for timestamp in t:
		result.append(v_min + a*timestamp + A*(math.exp(gamma * timestamp) * math.cos(omega*timestamp + phi) - math.cos(phi)))
	return np.array(result)
			
		
