#!/usr/bin/python3

from datalib import *

class TrajectoryExtender(object):
	"""docstring for TrajectoryExtender."""

	def __init__(self, **kwargs):
		self.timesteps = kwargs.get('timesteps', 10)
		self.delta_time = kwargs.get('delta_time', 0.01)

	def extend(self, trajectory, velocity_data, extend_before=True, extend_after=True):
		return trajectory.copy()
		
	def extend_velocity(self, velocity_data, extend_before=True, extend_after=True):
		return velocity_data.copy()
		
	def extra_time(self):
		return self.timesteps * self.delta_time
	
