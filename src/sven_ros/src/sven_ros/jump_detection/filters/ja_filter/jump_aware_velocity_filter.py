#!/usr/bin/python3

import functools
from sven_ros.jump_detection.datalib import *
from sven_ros.jump_detection.filters import *

class JumpAwareVelocityFilter(object):
	"""docstring for JumpAwareFilter."""

	def __init__(self, predictor, bounder, **kwargs):
		super(JumpAwareVelocityFilter, self).__init__()
		self.predictor = predictor
		self.bounder = bounder
		self.max_window_length = kwargs.get('max_window_length',20)
		self.window_length = 0
		self.data = DataSet()
		
	def copy(self):
		result = JumpAwareVelocityFilter(self.predictor.copy(), self.bounder.copy(), max_window_length=self.max_window_length)
		result.window_length = self.window_length
		result.data = self.data.copy()
		return result
		
	def update(self, datapoint, velocity):
		# Detect jumps
		jump_detected, info = self.detect_jump(datapoint, velocity)
		info.append(self.window_length)
		
		# Update window_length
		if jump_detected:
			self.window_length = 0
			self.predictor.reset()
			self.bounder.reset()
		else:
			self.window_length = min(self.window_length + 1, self.max_window_length)
			self.predictor.window_length = self.window_length
			self.bounder.window_length = self.window_length
			
		# Return result
		return jump_detected, info
	
	def detect_jump(self, datapoint, velocity):
		prediction, prediction_info = self.predictor.predict(datapoint)
		self.predictor.update(datapoint)
		bound, bound_info = self.bounder.bound(datapoint)
		self.bounder.update(datapoint)
		
		jump_detected = False
		if prediction.value is not None and bound.value is not None:
			if abs(prediction - velocity) > bound:
				jump_detected = True
		
		return jump_detected, [prediction, prediction_info, bound, bound_info]
	
	def reset(self):
		self.data.clear()
		self.window_length = 0
		self.predictor.reset()
		self.bounder.reset()

