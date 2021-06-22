#!/usr/bin/python3

from datalib import *

class PhaseWrapper(object):
	"""docstring for Joint."""

	def __init__(self, movement_primitive, **kwargs):
		self.movement_primitive = movement_primitive
		
		# Times of entire phase
		self.phase_starting_time = kwargs.get('phase_starting_time',0)
		self.phase_ending_time = kwargs.get('phase_ending_time',0)
		
		# Times of usable phase data
		self.starting_time = kwargs.get('starting_time',0)
		self.ending_time = kwargs.get('ending_time',0)
		
		# Times of extended phase. MP is valid for these times.
		self.extended_starting_time = kwargs.get('extended_starting_time',0)
		self.extended_ending_time = kwargs.get('extended_ending_time',0)
		
		# Next and previous phase. Time difference between phases.
		self.previous_ending_time = kwargs.get('previous_ending_time',None)
		self.next_starting_time = kwargs.get('next_starting_time',None)
		
		# Shift over time for evaluation.
		self.time_shift = kwargs.get('time_shift')
		
	def get_phase_start_end(self):
		return self.phase_starting_time + self.time_shift, self.phase_ending_time + self.time_shift
		
	def get_start_end(self):
		return self.starting_time + self.time_shift, self.ending_time + self.time_shift
		
	def get_extended_start_end(self):
		return self.extended_starting_time + self.time_shift, self.extended_ending_time + self.time_shift
		
	def evaluate(self, time, derivative=0, **kwargs):
		if not isinstance(time, list):
			time = time - self.time_shift
		else:
			time = time.copy()
			for i in range(len(time)):
				time[i] = time[i] - self.time_shift
			
		return self.movement_primitive.evaluate(time, derivative, **kwargs)
		
	def align_time(self, time_shift):
		self.time_shift = time_shift
		
