#!/usr/bin/python3

class Filter(object):
	"""docstring for Filter."""

	def __init__(self, config):
		self.config = config

	def filter(self, data, window_length=None):
		return None

	def est_vel(self, data, window_length=None):
		return None


class FilterConfiguration(object):
	"""docstring for FilterConfiguration."""

	def __init__(self, window_length):
		self.window_length = window_length
