#!/usr/bin/python3

import numpy as np
from .filter import *
from datalib import *

class LeastSquaresFilter(Filter):
	"""docstring for Filter."""
	
	def __init__(self, **kwargs):
		super(LeastSquaresFilter, self).__init__(**kwargs)
		self.order = kwargs.get('order',3)
		
	def copy(self):
		return LeastSquaresFilter(window_length=self.window_length, order=self.order)

	def filter(self, data, window_length=None):
		if window_length is None:
			window_length = self.window_length

		result = DataSet()
		coefs_list = []
		for i in range(len(data)):
			start = max(0, i - window_length)
			end = i + 1
			order = min(end - start - 1, self.order)
			if order == 0:
				result.append(data[i].copy())
				continue
			subset = data[start:end]
			x = []
			y = []
			for j in subset:
				x.append(j.time)
				y.append(j.value)
			coefs = np.polyfit(x, y, order)
			value = 0
			time = data[i].time
			for j in range(len(coefs)):
				coef = coefs[j]
				value = value + coef * (time ** (len(coefs) - j - 1))
			datapoint = DataPoint(data[i].timestamp, value)
			datapoint.time = data[i].time
			result.append(datapoint)
			coefs_list.append(coefs)
		return result, [coefs_list]

