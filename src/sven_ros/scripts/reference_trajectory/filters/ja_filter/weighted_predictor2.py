#!/usr/bin/python3

import numpy as np
from .base_predictor import *

class WeightedPredictor2(BasePredictor):
	"""docstring for Predictor."""

	def __init__(self, **kwargs):
		super(WeightedPredictor2, self).__init__(**kwargs)
		self.less_valued_datapoints = kwargs.get('less_valued_datapoints',3)
		self.remove_indices = kwargs.get('remove_indices',3)
		
	def copy(self):
		return WeightedPredictor2(order=self.order,less_valued_datapoints=self.less_valued_datapoints,remove_indices=self.remove_indices)

	def predict(self, data, window_length, time, **kwargs):
		order = min(min(window_length, self.order), len(data) - self.remove_indices)
		if order < self.order:
			return None, []
		
		x = []
		y = []
		w = []
		
		w_value = 1
		max_w_value = int(min((len(data) - self.remove_indices) / 2, self.less_valued_datapoints))
		for i in range(len(data) - self.remove_indices):
			x.append(data[i].time)
			y.append(data[i].value)
			w.append(np.sqrt(w_value))
			
			if i < max_w_value:
				w_value += 1
			elif i > len(data) - self.remove_indices - max_w_value - 1:
				w_value -= 1
			
		coefs = np.polyfit(x, y, order, w=w)
		value = 0
		for j in range(len(coefs)):
			coef = coefs[j]
			value = value + coef * (time ** (len(coefs) - j - 1))
		return value, [coefs]

