#!/usr/bin/python3

import numpy as np
from datalib.datapoint import DataPoint
from datalib.dataset import DataSet

class VelocityEstimator():
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)		

	def estimate(self, data, window_length=None):
		return None

