#!/usr/bin/python3

import numpy as np
from datalib import *
from .extender import *

class ConstantVelocityExtender(TrajectoryExtender):
	"""docstring for TrajectoryExtender."""
	
	def __init__(self, **kwargs):
		super(ConstantVelocityExtender, self).__init__(**kwargs)

	def extend(self, trajectory, velocity_data, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[]):
		if len(velocity_data) > 0:
			vel_start = velocity_data[0].value
			vel_end = velocity_data[-1].value
		else:
			vel_start = None
			vel_end = None
			
		result = DataSet(timefactor=trajectory.timefactor)
		
		if vel_start is not None and extend_before:
		
			starting_time = trajectory[0].time
			if len(extended_times_before) > 0:
				starting_time = extended_times_before[0]
				
			times = np.arange(starting_time - self.timesteps * self.delta_time, starting_time, self.delta_time).tolist()
			times.extend(extended_times_before.copy())
		
			for i in range(len(times)):
				timestamp = trajectory[0].timestamp - (trajectory[0].time - times[i]) * trajectory.timefactor
				value = trajectory[0].value - (trajectory[0].time - times[i]) * vel_start
				datapoint = DataPoint(timestamp, value)
				datapoint.time = times[i]
				result.append(datapoint)
		
		for i in trajectory:
			result.append(i.copy())
			
		if vel_end is not None and extend_after:
		
			starting_time = trajectory[-1].time
			if len(extended_times_after) > 0:
				starting_time = extended_times_after[-1]
			
			times = extended_times_after.copy()
			times.extend(np.arange(starting_time + self.delta_time, starting_time + (self.timesteps + 1) * self.delta_time, self.delta_time).tolist())
			
			for i in range(len(times)):
				timestamp = trajectory[-1].timestamp + (times[i] - trajectory[-1].time) * trajectory.timefactor
				value = trajectory[-1].value + (times[i] - trajectory[-1].time) * vel_end
				datapoint = DataPoint(timestamp, value)
				datapoint.time = times[i]
				result.append(datapoint)

		return result
		
	def extend_velocity(self, velocity_data, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[]):
	
		if len(velocity_data) > 0:
			vel_start = velocity_data[0].value
			vel_end = velocity_data[-1].value
		else:
			vel_start = None
			vel_end = None
			
		result = DataSet(timefactor=velocity_data.timefactor)
		
		if vel_start is not None and extend_before:
		
			starting_time = velocity_data[0].time
			if len(extended_times_before) > 0:
				starting_time = extended_times_before[0]
				
			times = np.arange(starting_time - self.timesteps * self.delta_time, starting_time, self.delta_time).tolist()
			times.extend(extended_times_before.copy())
		
			for i in range(len(times)):
				timestamp = velocity_data[0].timestamp - (velocity_data[0].time - times[i]) * velocity_data.timefactor
				value = vel_start
				datapoint = DataPoint(timestamp, value)
				datapoint.time = times[i]
				result.append(datapoint)
		
		for i in velocity_data:
			result.append(i.copy())
			
		if vel_end is not None and extend_after:
		
			starting_time = velocity_data[-1].time
			if len(extended_times_after) > 0:
				starting_time = extended_times_after[-1]
			
			times = extended_times_after.copy()
			times.extend(np.arange(starting_time + self.delta_time, starting_time + (self.timesteps + 1) * self.delta_time, self.delta_time).tolist())
			
			for i in range(len(times)):
				timestamp = velocity_data[-1].timestamp + (times[i] - velocity_data[-1].time) * velocity_data.timefactor
				value = vel_end
				datapoint = DataPoint(timestamp, value)
				datapoint.time = times[i]
				result.append(datapoint)

		return result
	
