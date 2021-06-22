#!/usr/bin/python3

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
		
			for i in range(self.timesteps):
				
				timestamp = trajectory[0].timestamp - (self.timesteps - i) * self.delta_time * trajectory.timefactor
				value = trajectory[0].value - (self.timesteps - i) * self.delta_time * vel_start
				datapoint = DataPoint(timestamp, value)
				datapoint.time = trajectory[0].time - (self.timesteps - i) * self.delta_time
				result.append(datapoint)
		
		for i in trajectory:
			result.append(i.copy())
			
		if vel_end is not None and extend_after:
			for i in range(self.timesteps):
				timestamp = trajectory[-1].timestamp + (i+1) * self.delta_time * trajectory.timefactor
				value = trajectory[-1].value + (i+1) * self.delta_time * vel_start
				datapoint = DataPoint(timestamp, value)
				datapoint.time = trajectory[-1].time + (i+1) * self.delta_time
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
		
		# TODO: deleted after and deleted before
		
		if vel_start is not None and extend_before:
			for i in range(self.timesteps):
				timestamp = velocity_data[0].timestamp - (self.timesteps - i) * self.delta_time * velocity_data.timefactor
				value = vel_start
				datapoint = DataPoint(timestamp, value)
				datapoint.time = velocity_data[0].time - (self.timesteps - i) * self.delta_time
				result.append(datapoint)
		
		for i in velocity_data:
			result.append(i.copy())
			
		if vel_end is not None and extend_after:
			for i in range(self.timesteps):
				timestamp = velocity_data[-1].timestamp + (i+1) * self.delta_time * velocity_data.timefactor
				value = vel_end
				datapoint = DataPoint(timestamp, value)
				datapoint.time = velocity_data[-1].time + (i+1) * self.delta_time
				result.append(datapoint)

		return result
	
