#!/usr/bin/python3

from datalib import *

class TrajectoryExtender(object):
	"""docstring for TrajectoryExtender."""

	def __init__(self, config):
		self.config = config

	def extend(self, trajectory):
		return None
		
class TrajectoryExtenderConfiguration(object):
	
	def __init__(self, timesteps=10, delta_time=0.1):
		self.timesteps = timesteps
		self.delta_time = delta_time
		
	
