#!/usr/bin/python3

from sven_ros.jump_detection.datalib import *
import numpy as np

class NumericalBounder(object):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		self.bound_ = kwargs.get('bound',[10000])
		self.window_length = kwargs.get('window_length',10)
		self.data = DataSet()
		
	def copy(self):
		result = NumericalBounder(bound=self.bound_, window_length=self.window_length)
		result.data = self.data.copy()
		return result

	def bound(self, datapoint):
		value = None
		subset = self.data[-self.window_length:]
		if len(subset) > 0:
			t_mean = (datapoint.time - subset[0].time) / len(subset)
			value = abs(np.polyval(self.bound_, t_mean))
		else:
			t_mean = None
		return DataPoint(datapoint.time, value), [t_mean]
		
	def set_bound(self, bound):
		self.bound_ = bound
		
	def update(self, datapoint):
		self.data.append(datapoint)
		while len(self.data) > self.window_length:
			self.data.pop(0)
		return None, []
		
	def reset(self):
		self.data.clear()

