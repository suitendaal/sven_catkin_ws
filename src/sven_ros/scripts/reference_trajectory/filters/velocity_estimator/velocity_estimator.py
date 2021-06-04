#!/usr/bin/python3

import numpy as np
from datalib.datapoint import DataPoint
from datalib.dataset import DataSet

class VelocityEstimator():
	"""docstring for Filter."""

	def __init__(self, config):
		self.config = config

	def estimate(self, data, window_length=None):
		return None


class VelocityEstimatorConfiguration():
	"""docstring for FilterConfiguration."""

	def __init__(self, window_length):
		self.window_length = window_length

