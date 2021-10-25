#!/usr/bin/python3

from .extender import *
import numpy as np
import math
import scipy.optimize as optimization

class ConstantVelocityExtender(Extender):

	def __init__(self, frequency, impact_duration=0, timespan=0, bounds=None):
		super().__init__(frequency, impact_duration, timespan)
		self.bounds = bounds
		if self.bounds is None:
			self.bounds = ((-np.inf, 0, -np.log(10)/((3/2)*self.impact_duration/3), -np.pi, (2/3)*omega), (np.inf, np.inf, 0, np.pi, (3.2)*omega))

	def calc_post_impact_velocity(self, phase, trajectory_handle):
		result = []
		velocity_data = PositionDataSet()
		fitted_velocity_data = PositionDataSet()
		fitted_linear_velocity_data = PositionDataSet()
		
		starting_index = trajectory_handle.trimmed_post_impact_index(phase)
		ending_index = self.post_impact_starting_index(phase, trajectory_handle)
		for datapoint in trajectory_handle.trajectory_data[starting_index:ending_index]:
			velocity_data.append(PositionDataPoint(datapoint.time, datapoint.value[1]))
			
		time_shift = velocity_data[0].time
		velocity_data.align_time()
		
		omega = 2*math.pi / (self.impact_duration/3)
		p0 = [0, 1/100, 0, 0, omega]
		bounds = self.bounds
		
		fitted_velocity_values = []
		fitted_linear_velocity_values = []
		
		for j in range(3):
			ante_impact_velocity = velocity_data[0].value[j]
			func = lambda t, a, A, gamma, phi, omega : fitting_func(t, a, A, gamma, omega, phi, ante_impact_velocity)
			
			p, pcov = optimization.curve_fit(func, np.array(velocity_data.time), np.array(velocity_data.get_index(j).value), p0=p0,bounds=bounds, maxfev=1000000)
			
			fitted_velocity_values.append(func(velocity_data.time, p[0], p[1], p[2], p[3], p[4]))
			fitted_linear_velocity_values.append(func(velocity_data.time, p[0], 0, 0, 0, 0))
			
			for k in range(len(fitted_linear_velocity_values[j])):
				fitted_linear_velocity_values[j][k] -= p[1] * math.cos(p[3])
		
		for k in range(len(velocity_data)):
			fitted_velocity = []
			fitted_linear_velocity = []
			for j in range(3):
				fitted_velocity.append(fitted_velocity_values[j][k])
				fitted_linear_velocity.append(fitted_linear_velocity_values[j][k])
			fitted_velocity_data.append(PositionDataPoint(velocity_data[k].time + time_shift, fitted_velocity))
			fitted_linear_velocity_data.append(PositionDataPoint(velocity_data[k].time + time_shift, fitted_linear_velocity))
		
		velocity_data.align_time(time_shift)
	
		return fitted_linear_velocity_data[-1].value, velocity_data, fitted_velocity_data, fitted_linear_velocity_data

	def extend_before(self, phase, trajectory_handle, trajectory):
		times = self.get_times_before(trajectory)
		result = DataSet()
		
		starting_time = trajectory[0].time
		position_value = trajectory[0].value[0]
		velocity_value, velocity_data, fitted_velocity_data, fitted_linear_velocity_data = self.calc_post_impact_velocity(phase, trajectory_handle)
		
		for t in times:
			value = []
			for i in range(len(trajectory[0].value)):
				value.append(None)
			
			for i in range(len(trajectory[0].value)):
				index = len(trajectory[0].value) - 1 - i
				if index > 1:
					tmp = []
					for j in trajectory[0].value[index]:
						tmp.append(0)
					value[index] = tmp
				elif index == 1:
					value[index] = velocity_value.copy()
				else:
					tmp = []
					for j in range(len(velocity_value)):
						tmp.append(position_value[j] - (starting_time - t) * velocity_value[j])
					value[index] = tmp
			result.append(DataPoint(t, value))
		
		for datapoint in trajectory:
			result.append(datapoint)
		
		return result
		
	def extend_after(self, phase, trajectory_handle, trajectory):
		times = self.get_times_after(trajectory)
		
		starting_time = trajectory[-1].time
		position_value = trajectory[-1].value[0]
		velocity_value = trajectory[-1].value[1]
		
		for t in times:
			value = []
			for i in range(len(trajectory[0].value)):
				value.append(None)
			
			for i in range(len(trajectory[-1].value)):
				index = len(trajectory[-1].value) - 1 - i
				if index > 1:
					tmp = []
					for j in trajectory[-1].value[index]:
						tmp.append(0)
					value[index] = tmp
				elif index == 1:
					value[index] = velocity_value.copy()
				else:
					tmp = []
					for j in range(len(velocity_value)):
						tmp.append(position_value[j] + (t - starting_time) * velocity_value[j])
					value[index] = tmp
			trajectory.append(DataPoint(t, value))
		
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
	
