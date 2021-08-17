#!/usr/bin/python3

from .datapoint import *

class FunctionPoint(DataPoint):
	"""docstring for DataPoint."""

	def evaluate(self, time=None):
		if time is None:
			time = self.time
		
		result = self.copy()
		if self.value is not None:
			result.value = self.value(time)
		else:
			result.value = None
		return result
		
	def copy(self):
		result = FunctionPoint(self.time, self.value)
		return result

