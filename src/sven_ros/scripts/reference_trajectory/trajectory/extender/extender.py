#!/usr/bin/python3

from datalib import *

class TrajectoryExtender(object):
	"""docstring for TrajectoryExtender."""

	def __init__(self, **kwargs):
		self.timesteps = kwargs.get('timesteps', 10)
		self.delta_time = kwargs.get('delta_time', 0.01)

	def extend(self, trajectory):
		return None
	
