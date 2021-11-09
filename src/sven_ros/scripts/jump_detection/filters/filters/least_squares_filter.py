#!/usr/bin/python3

import numpy as np
from .filter import *
from datalib import *

class LeastSquaresFilter(Filter):
	"""docstring for Filter."""
	
	def __init__(self, **kwargs):
		super(LeastSquaresFilter, self).__init__(**kwargs)
		self.order = kwargs.get('order',3)
		
	def copy(self):
		result = LeastSquaresFilter(window_length=self.window_length, order=self.order)
		result.data = self.data.copy()
		return result

	def filter(self, datapoint):
		subset = self.data[-self.window_length:]
		subset.append(datapoint)
		if not self.enough_data(len(subset)):
			return DataPoint(datapoint.time, None), []
		
		x = subset.time
		time_shift = x[0]
		time = datapoint.time - time_shift
		for i in range(len(x)):
			x[i] = x[i] - time_shift
		y = subset.value
		coefs = np.polyfit(x,y,self.order)
		value = np.polyval(coefs, time)
			
		return DataPoint(datapoint.time, value), coefs
		
	def predict(self, datapoint):
		subset = self.data[-self.window_length:]
		if not self.enough_data(len(subset)):
			return DataPoint(datapoint.time, None), []
		
		x = subset.time
		time_shift = x[0]
		time = datapoint.time - time_shift
		for i in range(len(x)):
			x[i] = x[i] - time_shift
		y = subset.value
		coefs = np.polyfit(x,y,self.order)
		value = np.polyval(coefs, time)
			
		return DataPoint(datapoint.time, value), coefs
		
	# Returns if there is enough data to fit a polynomial
	def enough_data(self, data_length):
#		return self.order + 1 + 1 <= data_length
		return self.order + 1 <= data_length

