#!/usr/bin/python3

import numpy as np
from .base_bounder import *

# Delta acceleration bounder
class AccBounder(BaseBounder):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
	
		# Default bound
		self.bound = kwargs.get('bound',0.01)
		self.acc_bound = kwargs.get('acc_bound',200)
		self.order = kwargs.get('order',3)
		
	def copy(self):
		return AccBounder(bound=self.bound, acc_bound=self.acc_bound)

	def calc_bound(self, data, window_length, time, **kwargs):
		
		order = min(window_length, self.order)
		if order == 0:
			return self.bound, []
			
		
		acc = np.mean(data.diff().diff().values())
		dt = time - data[-1].time

		return self.acc_bound * dt**2, []

