#!/usr/bin/python3

import numpy as np
from datalib import *
from .extender import *

class ConstantVelocityExtender(TrajectoryExtender):
	"""docstring for TrajectoryExtender."""
	
	def __init__(self, **kwargs):
		super(ConstantVelocityExtender, self).__init__(**kwargs)
		
	def copy(self):
		return ConstantVelocityExtender(timesteps=self.timesteps, delta_time=self.delta_time)

	def extend(self, trajectory, velocity=0, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[], **kwargs):
		result = DataSet()
		
		if extend_before:
		
			starting_time = trajectory[0].time
			if len(extended_times_before) > 0:
				extended_times_before.sort()
				starting_time = extended_times_before[0]
				
			times = np.arange(starting_time - self.timesteps * self.delta_time, starting_time, self.delta_time).tolist()
			times.extend(extended_times_before.copy())
		
			for i in range(len(times)):
				value = trajectory[0].value - (trajectory[0].time - times[i]) * velocity
				datapoint = DataPoint(times[i], value)
				result.append(datapoint)
		
		for i in trajectory:
			result.append(i.copy())
			
		if extend_after:
		
			starting_time = trajectory[-1].time
			if len(extended_times_after) > 0:
				extended_times_after.sort()
				starting_time = extended_times_after[-1]
			
			times = extended_times_after.copy()
			times.extend(np.arange(starting_time + self.delta_time, starting_time + (self.timesteps + 1) * self.delta_time, self.delta_time).tolist())
			
			for i in range(len(times)):
				value = trajectory[-1].value + (times[i] - trajectory[-1].time) * velocity
				datapoint = DataPoint(times[i], value)
				result.append(datapoint)

		return result
		
	def extend_velocity(self, velocity_data, velocity=0, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[], **kwargs):
	
		result = DataSet()
		
		if extend_before:
		
			starting_time = velocity_data[0].time
			if len(extended_times_before) > 0:
				extended_times_before.sort()
				starting_time = extended_times_before[0]
				
			times = np.arange(starting_time - self.timesteps * self.delta_time, starting_time, self.delta_time).tolist()
			times.extend(extended_times_before.copy())
		
			for i in range(len(times)):
				value = velocity
				datapoint = DataPoint(times[i], value)
				result.append(datapoint)
		
		for i in velocity_data:
			result.append(i.copy())
			
		if extend_after:
		
			starting_time = velocity_data[-1].time
			if len(extended_times_after) > 0:
				extended_times_after.sort()
				starting_time = extended_times_after[-1]
			
			times = extended_times_after.copy()
			times.extend(np.arange(starting_time + self.delta_time, starting_time + (self.timesteps + 1) * self.delta_time, self.delta_time).tolist())
			
			for i in range(len(times)):
				value = velocity
				datapoint = DataPoint(times[i], value)
				result.append(datapoint)

		return result
	
