#!/usr/bin/python3

from .datapoint import *

class FunctionPoint(DataPoint):
	"""docstring for DataPoint."""

	def evaluate(self):
		result = self.copy()
		if self.value is not None:
			result.value = self.value(self.time)
		else:
			result.value = (None, [])
		return result
		
	def copy(self):
		result = FunctionPoint(self.timestamp, self.value)
		result.time = self.time
		return result

