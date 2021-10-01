#!/usr/bin/python3

from .jump_aware_filter import *

class JumpAwareExternalForceFilter(JumpAwareFilter):
	"""docstring for JumpAwareFilter."""

	def __init__(self, predictor, bounder, **kwargs):
		super(JumpAwareExternalForceFilter, self).__init__(predictor, bounder, **kwargs)
		
	def copy(self):
		result = JumpAwareExternalForceFilter(self.predictor.copy(), self.bounder.copy(), max_window_length=self.max_window_length)
		result.window_length = self.window_length
		result.data = self.data.copy()
		return result
		
	def update(self, datapoint):
		# Detect jumps
		jump_detected, info = self.detect_jump(datapoint)
		info.append(self.window_length)
		info.append(jump_detected)
		
		# Update window_length
		if jump_detected:
			self.window_length = 0
			self.predictor.reset()
			self.bounder.reset()
			self.data.clear()
		else:
			self.window_length = min(self.window_length + 1, self.max_window_length)
			self.predictor.window_length = self.window_length
			self.predictor.update(datapoint)
			self.bounder.window_length = self.window_length
			self.bounder.update(datapoint)
			self.data.append(datapoint)
			
		# Return result
		return (jump_detected and datapoint.value > info[0]), info
	
	def detect_jump(self, datapoint):
		prediction, prediction_info = self.predictor.predict(datapoint)
		bound, bound_info = self.bounder.bound(datapoint)
		
		jump_detected = False
		if prediction.value is not None and bound.value is not None:
			if abs(prediction - datapoint) > bound:
				jump_detected = True
		
		return jump_detected, [prediction, bound, prediction_info, bound_info]

