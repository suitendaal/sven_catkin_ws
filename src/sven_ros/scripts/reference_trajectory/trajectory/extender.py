#!/usr/bin/python3

from datalib.dataset import DataSet
from datalib.datapoint import DataPoint

class TrajectoryExtender(object):
	"""docstring for TrajectoryExtender."""

	def __init__(self, config):
		self.config = config

	def extend_const_vel(self, trajectory):
		result = DataSet(timefactor=trajectory.timefactor)
		vel_data = self.config.velocity_estimator(trajectory)
		vel_start = vel_data[0]
		vel_end = vel_data[-1]
		
		for i in range(self.config.timesteps):
			timestamp = trajectory[0].timestamp - (self.config.timesteps - i) * self.config.delta_time * trajectory.timefactor
			value = trajectory[0].value - (self.config.timesteps - i) * self.config.delta_time * vel_start
			result.append(DataPoint(timestamp, value))
		
		for i in trajectory:
			result.append(DataPoint(i.timestamp, i.value))
			
		for i in range(self.config.timesteps):
			timestamp = trajectory[-1].timestamp + (i+1) * self.config.delta_time * trajectory.timefactor
			value = trajectory[-1].value + (i+1) * self.config.delta_time * vel_start
			result.append(DataPoint(timestamp, value))
			
		result.align_time(trajectory[0].time)

		return result

def main():
	
	
