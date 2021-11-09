#!/usr/bin/python3

from datalib import *
from .constant_bounder import *

class ForceDerivativeBounder(ConstantBounder):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		super(ForceDerivativeBounder, self).__init__(**kwargs)
		self.data = DataSet()
		
	def copy(self):
		result = ForceDerivativeBounder(bound=self.bound_, window_length=self.window_length)
		result.data = self.data.copy()
		return result

	def bound(self, datapoint):
		value = None
		subset = self.data[-self.window_length:]
		if self.bound_ is not None and len(subset) > 0:
			value = (datapoint.time - subset[-1].time) * self.bound_
		return DataPoint(datapoint.time, value), []
		
	def set_bound(self, bound):
		self.bound_ = bound
		
	def update(self, datapoint):
		self.data.append(datapoint)
		while len(self.data) > self.window_length:
			self.data.pop(0)
		return None, []
		
	def reset(self):
		self.data.clear()
		
