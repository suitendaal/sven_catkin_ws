#!/usr/bin/python3

import functools
from datalib import *
from .multi_jump_aware_filter import *

class MultiExternalForceJumpAwareFilter(MultiJumpAwareFilter):
	"""docstring for JumpAwareFilter."""
		
	def copy(self):
		result = MultiExternalForceJumpAwareFilter(self.predictor.copy(), self.bounder.copy(), max_window_length=self.max_window_length)
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

