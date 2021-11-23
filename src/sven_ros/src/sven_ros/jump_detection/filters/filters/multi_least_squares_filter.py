#!/usr/bin/python3

import numpy as np
from .least_squares_filter import *
from sven_ros.jump_detection.datalib import *

class MultiLeastSquaresFilter(LeastSquaresFilter):
	"""docstring for Filter."""
	
	def copy(self):
		result = MultiLeastSquaresFilter(window_length=self.window_length, order=self.order)
		result.data = self.data.copy()
		return result

	def filter(self, datapoint):
		subset = self.data[-self.window_length:]
		subset.append(datapoint)
		if not self.enough_data(subset, datapoint):
			return DataPoint(datapoint.time, None), []
		
		x = subset.time
		time_shift = x[0]
		time = datapoint.time - time_shift
		for i in range(len(x)):
			x[i] = x[i] - time_shift
		
		coefs = []
		value = []
		for j in range(len(datapoint.value)):
			y = subset.get_index(j).value
			coefs.append(np.polyfit(x,y,self.order))
			value.append(np.polyval(coefs[j], time))
			
		return DataPoint(datapoint.time, value), coefs
		
	def predict(self, datapoint):
		subset = self.data[-self.window_length:]
		if not self.enough_data(subset, datapoint):
			return DataPoint(datapoint.time, None), []
		
		x = subset.time
		time_shift = x[0]
		time = datapoint.time - time_shift
		for i in range(len(x)):
			x[i] = x[i] - time_shift
		
		coefs = []
		value = []
		for j in range(len(datapoint.value)):
			y = subset.get_index(j).value
			coefs.append(np.polyfit(x,y,self.order))
			value.append(np.polyval(coefs[j], time))
			
		return DataPoint(datapoint.time, value), coefs
		
