#!/usr/bin/python3

class BaseBounder(object):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		self.bound = kwargs.get('bound',0.01)
		
	def copy(self):
		return BaseBounder(bound=self.bound)

	def calc_bound(self, data, window_length, time, **kwargs):
		return self.bound, []

