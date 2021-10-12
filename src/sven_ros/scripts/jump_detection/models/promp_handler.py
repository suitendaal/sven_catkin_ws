#!/usr/bin/python3

import json
from datalib import *
from trajectory import *

class ProMPHandler(object):
	"""docstring for Joint."""

	def __init__(self, movement_primitive=None, **kwargs):
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
		self.time_shift = kwargs.get('time_shift',0)
		
	def get_phase_start_end(self):
		return self.phase_starting_time + self.time_shift, self.phase_ending_time + self.time_shift
		
	def get_start_end(self):
		return self.starting_time + self.time_shift, self.ending_time + self.time_shift
		
	def get_extended_start_end(self):
		return self.extended_starting_time + self.time_shift, self.extended_ending_time + self.time_shift
		
	def evaluate(self, time, derivative=0, **kwargs):
		via_points = kwargs.get('via_points',DataSet()).copy()
		for via_point in via_points:
			via_point.time = via_point.time - self.time_shift
		kwargs['via_points'] = via_points
		
		if not isinstance(time, list):
			time = time - self.time_shift
		else:
			time = time.copy()
			for i in range(len(time)):
				time[i] = time[i] - self.time_shift
			
		return self.movement_primitive.evaluate(time, derivative, **kwargs)
		
	def align_time(self, time_shift):
		self.time_shift = time_shift
		
	def to_dict(self):
		json_object = dict()
		
		# Times of entire phase
		json_object['phase_starting_time'] = self.phase_starting_time
		json_object['phase_ending_time'] = self.phase_ending_time
		
		# Times of usable phase data
		json_object['starting_time'] = self.starting_time
		json_object['ending_time'] = self.ending_time
		
		# Times of extended phase. MP is valid for these times.
		json_object['extended_starting_time'] = self.extended_starting_time
		json_object['extended_ending_time'] = self.extended_ending_time
		
		# Next and previous phase. Time difference between phases.
		json_object['previous_ending_time'] = self.previous_ending_time
		json_object['self.next_starting_time'] = self.next_starting_time
		
		# Shift over time for evaluation.
		json_object['time_shift'] = self.time_shift
		
		# Movement primitive
		json_object['movement_primitive'] = self.movement_primitive.to_dict()
		
		return json_object
		
	def from_dict(self, json_object):
		self.phase_starting_time = json_object['phase_starting_time']
		self.phase_ending_time = json_object['phase_ending_time']
		
		# Times of usable phase data
		self.starting_time = json_object['starting_time']
		self.ending_time = json_object['ending_time']
		
		# Times of extended phase. MP is valid for these times.
		self.extended_starting_time = json_object['extended_starting_time']
		self.extended_ending_time = json_object['extended_ending_time']
		
		# Next and previous phase. Time difference between phases.
		self.previous_ending_time = json_object['previous_ending_time']
		self.next_starting_time = json_object['self.next_starting_time']
		
		# Shift over time for evaluation.
		self.time_shift = json_object['time_shift']
		
		# Movement primitive
		basis_functions = self.basis_functions_from_dict(json_object['movement_primitive']['basis_functions'])
		self.movement_primitive = ProMP(basis_functions)
		self.movement_primitive.from_dict(json_object['movement_primitive'])
		
	def basis_functions_from_dict(self, json_object):
		basis_functions = []
		for i in json_object:
			if i['type'] == 'RadialBasisFunction':
				basis_function = RadialBasisFunction()
				basis_function.from_dict(i)
				basis_functions.append(basis_function)
		return basis_functions
		
		
