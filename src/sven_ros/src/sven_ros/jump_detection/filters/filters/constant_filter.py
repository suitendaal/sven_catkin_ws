#!/usr/bin/python3

import numpy as np
from .filter import *
from sven_ros.jump_detection.datalib import *

class ConstantFilter(Filter):
	"""docstring for Filter."""
	
	def __init__(self, **kwargs):
		super(ConstantFilter, self).__init__(**kwargs)
		self.value = kwargs.get('value',0)
		
	def copy(self):
		result = ConstantFilter(window_length=self.window_length, value=self.value)
		result.data = self.data.copy()
		return result

	def filter(self, datapoint):
		subset = self.data[-self.window_length:]
		subset.append(datapoint)
		if not self.enough_data(len(subset)):
			return DataPoint(datapoint.time, None), []
			
		return DataPoint(datapoint.time, self.value), [self.value]
		
	def predict(self, datapoint):
		subset = self.data[-self.window_length:]
		if not self.enough_data(len(subset)):
			return DataPoint(datapoint.time, None), []
			
		return DataPoint(datapoint.time, self.value), [self.value]
		
	# Returns if there is enough data
	def enough_data(self, data_length):
		return True

