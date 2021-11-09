#!/usr/bin/python3

import numpy as np
from .least_squares_filter import *
from datalib import *

class ForceDerivativePredictor(Filter):
	"""docstring for Filter."""
	
	def __init__(self, **kwargs):
		super(ForceDerivativePredictor, self).__init__(**kwargs)
		
	def copy(self):
		result = ForceDerivativePredictor(window_length=self.window_length)
		result.data = self.data.copy()
		return result
		
	def predict(self, datapoint):
		subset = self.data[-self.window_length:]
		if not self.enough_data(len(subset)):
			return DataPoint(datapoint.time, None), []
			
		return DataPoint(datapoint.time, subset[-1].value), []
		
	# Returns if there is enough data to fit a polynomial
	def enough_data(self, data_length):
		return 1 <= data_length

