#!/usr/bin/python3

from .datapoint import *

class ViaPoint(DataPoint):
	"""docstring for DataPoint."""

	def __init__(self, timestamp, value, timefactor=1, derivative=0):
		super(ViaPoint, self).__init__(timestamp, value, timefactor)
		self.derivative = derivative

	def copy(self):
		result = ViaPoint(self.timestamp, self.value)
		result.time = self.time
		result.derivative = self.derivative
		return result

