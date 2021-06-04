#!/usr/bin/python3

from datalib import *
from .extender import *

class ConstantVelocityExtender(TrajectoryExtender):
	"""docstring for TrajectoryExtender."""
	
	def __init__(self, velocity_estimator, **kwargs):
		super(ConstantVelocityExtender, self).__init__(**kwargs)
		self.velocity_estimator = velocity_estimator

	def extend(self, trajectory):
		result = DataSet(timefactor=trajectory.timefactor)
		vel_data = self.velocity_estimator.estimate(trajectory)
		
		vel_start = 0
		vel_end = 0
		i = 0
		for i in range(len(vel_data)):
			if vel_data[i].value is not None:
				vel_start = vel_data[i].value
				break
		
		for i in range(len(vel_data)):
			if vel_data[-i-1].value is not None:
				vel_end = vel_data[i].value
				break
		
		for i in range(self.timesteps):
			timestamp = trajectory[0].timestamp - (self.timesteps - i) * self.delta_time * trajectory.timefactor
			value = trajectory[0].value - (self.timesteps - i) * self.delta_time * vel_start
			result.append(DataPoint(timestamp, value))
		
		for i in trajectory:
			result.append(DataPoint(i.timestamp, i.value))
			
		for i in range(self.timesteps):
			timestamp = trajectory[-1].timestamp + (i+1) * self.delta_time * trajectory.timefactor
			value = trajectory[-1].value + (i+1) * self.delta_time * vel_start
			result.append(DataPoint(timestamp, value))
			
		result.align_time(trajectory[0].time - result[0].time)

		return result		
	
