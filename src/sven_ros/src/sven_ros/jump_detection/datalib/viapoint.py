#!/usr/bin/python3

from .datapoint import *

class ViaPoint(DataPoint):
	"""docstring for DataPoint."""

	def __init__(self, time, value, derivative=0):
		super(ViaPoint, self).__init__(time, value)
		self.derivative = derivative

	def copy(self):
		result = ViaPoint(self.time, self.value)
		result.derivative = self.derivative
		return result

