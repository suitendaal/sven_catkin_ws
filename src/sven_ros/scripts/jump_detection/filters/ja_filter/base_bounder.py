#!/usr/bin/python3

from datalib import *

class BaseBounder(object):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		self.bound_ = kwargs.get('bound',0.01)
		
	def copy(self):
		return BaseBounder(bound=self.bound_)

	def bound(self, datapoint):
		return DataPoint(datapoint.time, self.bound_), []
		
	def update(self, datapoint):
		return None, []
		
	def reset(self):
		return

