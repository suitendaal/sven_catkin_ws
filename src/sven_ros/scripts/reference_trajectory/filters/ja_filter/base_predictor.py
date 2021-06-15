#!/usr/bin/python3

import numpy as np

class BasePredictor(object):
	"""docstring for Predictor."""

	def __init__(self, **kwargs):
		self.order = kwargs.get('order',3)
		
	def copy(self):
		return BasePredictor(order=self.order)

	def predict(self, data, window_length, time, **kwargs):
		order = min(len(data) - 1, self.order)
		if order == 0:
			return data[-1].value

		x = []
		y = []
		for j in data:
			x.append(j.time)
			y.append(j.value)
		coefs = np.polyfit(x, y, order)
		value = 0
		for j in range(len(coefs)):
			coef = coefs[j]
			value = value + coef * (time ** (len(coefs) - j - 1))
		#print("Coefs: ",coefs)
		return value, [coefs]

