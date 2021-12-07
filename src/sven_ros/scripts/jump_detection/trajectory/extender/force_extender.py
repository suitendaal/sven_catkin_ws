#!/usr/bin/python3

from .extender import *
import numpy as np
import math
import scipy.optimize as optimization

class ForceExtender(Extender):

	def __init__(self, frequency, impact_duration=0, timespan=0, bounds=None):
		super().__init__(frequency, impact_duration, timespan)
		self.bounds = bounds
		if self.bounds is None:
			omega = 2*math.pi / (self.impact_duration/3)
			self.bounds = ((-np.inf, 0, -np.log(10)/((3/2)*self.impact_duration/3), -np.pi, (2/3)*omega), (np.inf, np.inf, 0, np.pi, (3.2)*omega))

	def calc_post_impact_force(self, phase, trajectory_handle):
		result = []
		force_data = PositionDataSet()
		fitted_force_data = PositionDataSet()
		fitted_linear_force_data = PositionDataSet()
		
		starting_index = trajectory_handle.trimmed_post_impact_index(phase)
		ending_index = self.post_impact_starting_index(phase, trajectory_handle)
		for datapoint in trajectory_handle.trajectory_data[starting_index:ending_index]:
			force_data.append(PositionDataPoint(datapoint.time, datapoint.value[0]))
			
		time_shift = force_data[0].time
		force_data.align_time()
		
		omega = 2*math.pi / (self.impact_duration/3)
		p0 = [0, 1/100, 0, 0, omega]
		bounds = self.bounds
		
		fitted_force_values = []
		fitted_linear_force_values = []
		
		for j in range(3):
			ante_impact_force = force_data[0].value[j]
			func = lambda t, a, A, gamma, phi, omega : fitting_func(t, a, A, gamma, omega, phi, ante_impact_force)
			
			p, pcov = optimization.curve_fit(func, np.array(force_data.time), np.array(force_data.get_index(j).value), p0=p0,bounds=bounds, maxfev=1000000)
			
			fitted_force_values.append(func(force_data.time, p[0], p[1], p[2], p[3], p[4]))
			fitted_linear_force_values.append(func(force_data.time, p[0], 0, 0, 0, 0))
			
			for k in range(len(fitted_linear_force_values[j])):
				fitted_linear_force_values[j][k] -= p[1] * math.cos(p[3])
			
			result.append(lambda t, p=p.copy() : func(t - time_shift, p[0], 0, 0, 0, 0) - p[1] * math.cos(p[3]))
		
		for k in range(len(force_data)):
			fitted_force = []
			fitted_linear_force = []
			for j in range(3):
				fitted_force.append(fitted_force_values[j][k])
				fitted_linear_force.append(fitted_linear_force_values[j][k])
			fitted_force_data.append(PositionDataPoint(force_data[k].time + time_shift, fitted_force))
			fitted_linear_force_data.append(PositionDataPoint(force_data[k].time + time_shift, fitted_linear_force))
		
		force_data.align_time(time_shift)
	
		return result, force_data, fitted_force_data, fitted_linear_force_data

	def extend_before(self, phase, trajectory_handle, trajectory):
		times = self.get_times_before(trajectory)
		result = DataSet()
		
		starting_time = trajectory[0].time
		linear_force_functions, force_data, fitted_force_data, fitted_linear_force_data = self.calc_post_impact_force(phase, trajectory_handle)
		
		for t in times:
			value = []
			
			for j in range(len(trajectory[0].value[0])):
				force_func = linear_force_functions[j]
				value.append(force_func(np.array([t]))[0])
				
			result.append(DataPoint(t, [value]))
		
		for datapoint in trajectory:
			result.append(datapoint)
		
		return result
		
	def extend_after(self, phase, trajectory_handle, trajectory):
		times = self.get_times_after(trajectory)
		
		starting_time = trajectory[-1].time
		force_value = trajectory[-1].value[0]
		
		for t in times:
			value = force_value.copy()
			trajectory.append(DataPoint(t, [value]))
		
		return trajectory
		
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
	
