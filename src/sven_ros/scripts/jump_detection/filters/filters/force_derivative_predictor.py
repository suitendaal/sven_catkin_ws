#!/usr/bin/python3

import numpy as np
from .least_squares_filter import *
from datalib import *

class ForceDerivativePredictor(LeastSquaresFilter):
	"""docstring for Filter."""
	
	def __init__(self, **kwargs):
		super(ForceDerivativePredictor, self).__init__(**kwargs)
		self.time_window = kwargs.get('time_window',0.05)
		self.order = 0
		self.last_jump_time = None
		
	def copy(self):
		result = ForceDerivativePredictor(window_length=self.window_length, time_window=self.time_window)
		result.data = self.data.copy()
		return result
		
	# Returns if there is enough data to fit a polynomial
	def enough_data(self, subset, datapoint):
		if len(subset) == 0:
			return False
		return self.last_jump_time is None or self.time_window <= datapoint.time - self.last_jump_time
		
	def reset(self):
		if len(self.data) > 0:
			self.last_jump_time = self.data[-1].time
		super().reset()
		
	def update(self, datapoint):
		if self.last_jump_time is not None and self.last_jump_time >= datapoint.time:
			self.last_jump_time = None
		super().update(datapoint)

