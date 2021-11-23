#!/usr/bin/python3

from sven_ros.jump_detection.datalib import *

class ConstantBounder(object):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		self.bound_ = kwargs.get('bound',0.01)
		self.window_length = 0
		
	def copy(self):
		return ConstantBounder(bound=self.bound_)

	def bound(self, datapoint):
		return DataPoint(datapoint.time, self.bound_), []
		
	def set_bound(self, bound):
		self.bound_ = bound
		
	def update(self, datapoint):
		return None, []
		
	def reset(self):
		return

