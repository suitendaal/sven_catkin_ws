#!/usr/bin/python3

class Filter(object):
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)
		
	def copy(self):
		return Filter(window_length=self.window_length)

	def filter(self, data, window_length=None):
		return None, []

