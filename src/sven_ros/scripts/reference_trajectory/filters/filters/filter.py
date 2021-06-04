#!/usr/bin/python3

class Filter(object):
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)

	def filter(self, data, window_length=None):
		return None

	def est_vel(self, data, window_length=None):
		return None

