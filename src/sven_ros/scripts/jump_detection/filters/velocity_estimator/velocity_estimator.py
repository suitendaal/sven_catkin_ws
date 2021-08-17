#!/usr/bin/python3

import numpy as np
from datalib.datapoint import DataPoint
from datalib.dataset import DataSet

class VelocityEstimator():
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)
		self.data = DataSet()
		
	def copy(self):
		result = VelocityEstimator(window_length=self.window_length)
		result.data = self.data.copy()
		return result
		
	def reset(self):
		self.data.clear()
		
	def update(self, datapoint):
		# Estimate velocity
		velocity_estimation, info = self.estimate(datapoint)
		
		# Update data
		self.data.append(datapoint)
		
		# Return result
		return velocity_estimation, info

	def estimate(self, datapoint):
		return None, []

