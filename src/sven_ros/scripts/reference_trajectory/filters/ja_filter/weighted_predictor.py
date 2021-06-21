#!/usr/bin/python3

import numpy as np
from .base_predictor import *

class WeightedPredictor(BasePredictor):
	"""docstring for Predictor."""

	def __init__(self, **kwargs):
		super(WeightedPredictor, self).__init__(**kwargs)
		self.frequency = kwargs.get('frequency',3)
		
	def copy(self):
		return WeightedPredictor(order=self.order,frequency=self.frequency)

	def predict(self, data, window_length, time, **kwargs):
		order = min(int(window_length / self.frequency), self.order)
		if order < self.order:
			return None, []

		x = []
		y = []
		w = []
		j = 1
		for i in range(len(data)):
			if i % self.frequency == 0:
				x.append(data[i].time)
				y.append(data[i].value)
				w.append(1/j)
				j += 1
		coefs = np.polyfit(x, y, order, w=w)
		value = 0
		for j in range(len(coefs)):
			coef = coefs[j]
			value = value + coef * (time ** (len(coefs) - j - 1))
		return value, [coefs]

