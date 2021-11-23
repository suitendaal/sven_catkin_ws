#!/usr/bin/python3

import functools
from datalib import *
from .jump_aware_filter import *
import numpy as np

class MultiJumpAwareFilter(JumpAwareFilter):
	"""docstring for JumpAwareFilter."""
		
	def copy(self):
		result = MultiJumpAwareFilter(self.predictor.copy(), self.bounder.copy(), max_window_length=self.max_window_length)
		result.window_length = self.window_length
		result.data = self.data.copy()
		return result
	
	def detect_jump(self, datapoint):
		prediction, prediction_info = self.predictor.predict(datapoint)
		bound, bound_info = self.bounder.bound(datapoint)
		
		jump_detected = False
		if prediction.value is not None and bound.value is not None:
			if np.linalg.norm((prediction - datapoint).value) > bound:
				jump_detected = True
		
		return jump_detected, [prediction, bound, prediction_info, bound_info]

