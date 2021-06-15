#!/usr/bin/python3

import numpy as np
from datalib.datapoint import DataPoint
from datalib.dataset import DataSet

class VelocityEstimator():
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)
		
	def copy(self):
		return VelocityEstimator(window_length=self.window_length)

	def estimate(self, data, **kwargs):
		return None, []

