#!/usr/bin/python3

from datalib import *

class Filter(object):
	"""docstring for Filter."""

	def __init__(self, **kwargs):
		self.window_length = kwargs.get('window_length',10)
		self.data = DataSet()
		
	def copy(self):
		result = Filter(window_length=self.window_length)
		result.data = self.data.copy()
		return result
		
	def reset(self):
		self.data.clear()
		
	def update(self, datapoint):
		# Filter
		filtered_datapoint, info = self.filter(datapoint)
		
		# Update data
		self.data.append(datapoint)
		while len(self.data) > self.window_length:
			self.data.pop(0)
		
		# Return result
		return filtered_datapoint, info

	def filter(self, datapoint):
		return None, []

