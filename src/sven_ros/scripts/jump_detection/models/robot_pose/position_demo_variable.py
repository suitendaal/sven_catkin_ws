#!/usr/bin/python3

import math
import scipy.optimize as optimization
import numpy as np
from .orientation_demo_variable import *
import matplotlib.pyplot as plt

class PositionDemoVariable(OrientationDemoVariable):
	def __init__(self, data, derivative_data, impact_intervals, **kwargs):
		super(PositionDemoVariable, self).__init__(data, impact_intervals, **kwargs)
		self.derivative_data_ = derivative_data
		self.extended_derivative_ = None
		
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
		data = self.get_data(phase).copy()
		derivative_data = self.get_derivative_data(phase).copy()
		
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
		deleted_before, deleted_after = self.remove_indices(data, derivative_data, t_start, t_end, phase)
				
		# Update real times
		self.real_times_[phase] = [data[0].time + self.time_shift(phase), data[-1].time + self.time_shift(phase)]	
		
		# Initialize extended data
		extended_data = data.copy()
		extended_derivative_data = derivative_data.copy()
		
		# Extend data before
		if extend_before:
			extended_data = extender.extend(extended_data, velocity=post_impact_velocity, extend_before=extend_before, extended_times_before=deleted_before)
			extended_derivative_data = extender.extend_velocity(extended_derivative_data, velocity=post_impact_velocity, extend_before=extend_before, extended_times_before=deleted_before)
		
		# Extend data after
		if extend_after:
			extended_data = extender.extend(extended_data, velocity=ante_impact_velocity, extend_after=extend_after, extended_times_after=deleted_after)
			extended_derivative_data = extender.extend_velocity(extended_derivative_data, velocity=ante_impact_velocity, extend_after=extend_after, extended_times_after=deleted_after)
		
		self.extended_times_[phase] = [extended_data[0].time + self.time_shift(phase), extended_data[-1].time + self.time_shift(phase)]
		
#		plt.figure()
#		plt.plot(derivative_data.time, derivative_data.value)
#		plt.show()
		
		return extended_data, extended_derivative_data
		
	def remove_indices(self, data, derivative_data, t_start, t_end, phase):
		deleted_times_before = []
		deleted_times_after = []
		
		# Remove indices out of time range
		deleted_times_before.extend(self.remove_before(data, derivative_data, t_start, phase))
		deleted_times_after.extend(self.remove_after(data, derivative_data, t_end, phase))
			
		return deleted_times_before, deleted_times_after
		
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
		
	def get_extended_derivative(self, phase):
		if self.extended_derivative_ is not None and len(self.extended_derivative_) > phase:
			return self.extended_derivative_[phase]
		return self.get_data(phase)
		
	def calculate_post_impact_velocity(self, phase, return_evaluated_function=False):
		if phase == 0:
			return 0
		starting_index = self.get_starting_index(phase)
		ending_index = starting_index + 1
		for datapoint in self.get_derivative_data(phase):
			if datapoint.time < self.derivative_data_[starting_index].time + self.impact_duration:
				ending_index += 1
			else:
				break
		derivative_data = self.derivative_data_[starting_index:ending_index].copy()
		if len(derivative_data) < 5:
			return 0
		time_shift = derivative_data[0].time
		derivative_data.align_time(1)
		ante_impact_velocity = derivative_data[0].value
		
		return ante_impact_velocity

		func = lambda t, a, A, gamma, omega, phi : fitting_func(t, a, A, gamma, omega, phi, ante_impact_velocity)
		popt, pcov = optimization.curve_fit(func, np.array(derivative_data.time), np.array(derivative_data.value),maxfev=5000)
		result = func(0, popt[0], 0, 0, 0, 0) - popt[1]*math.cos(popt[4])
		
		if not return_evaluated_function:
			return result
			
		fitted_data = DataSet()
		fitted_data_values = func(derivative_data.time, popt[0], popt[1], popt[2], popt[3], popt[4])
		linearized_data = DataSet()
		linearized_data_values = func(derivative_data.time, popt[0], 0, 0, 0, 0)
		for i in range(len(fitted_data_values)):
			fitted_data.append(DataPoint(derivative_data[i].time + time_shift, fitted_data_values[i]))
			linearized_data_values.append(DataPoint(derivative_data[i].time + time_shift, linearized_data[i] - popt[1]*math.cos(popt[4])))
		
		return result, fitted_data, linearized_data
		
	def calculate_ante_impact_velocity(self, phase):
		if phase == self.n_phases - 1:
			return 0
		return self.get_derivative_data(phase)[-1].value
		
def fitting_func(t, a, A, gamma, omega, phi, v_min):
	result = []
	if not isinstance(t, (np.ndarray, np.generic)):
		if isinstance(t, list):
			t = np.array(t)
		else:
			t = np.array([t])
	
	for timestamp in t:
		result.append(v_min + a*timestamp + A*(math.exp(gamma * timestamp) * math.cos(omega*timestamp + phi) - math.cos(phi)))
	return np.array(result)
		
