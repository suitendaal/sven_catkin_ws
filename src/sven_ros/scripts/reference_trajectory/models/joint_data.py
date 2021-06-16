#!/usr/bin/python3

from datalib import *

class JointData(object):
	"""docstring for Joint."""

	def __init__(self, pos_data, **kwargs):
		self.pos_data = pos_data
		self.vel_data = kwargs.get('vel_data',DataSet())
		self.eff_data = kwargs.get('eff_data',DataSet())
		
		self.filtered_data = DataSet()
		self.velocity_estimation = DataSet()
		self.jumping_indexes = []
		
		self.info = []
		
		self.euler_vel = pos_data.diff()
		
	def detect_jumps(self, jump_detector):
		self.filtered_data, self.velocity_estimation, self.jumping_indexes, self.info = jump_detector.filter(self.pos_data)
		return self.jumps()
		
	def starting_time(self):
		result = 0
		if len(self.jumping_indexes) > 0:
			result = -self.pos_data[self.jumping_indexes[0]].time
		return result
		
	def starting_time_before(self):
		result = 0
		if len(self.jumping_indexes) > 0:
			result = -self.pos_data[self.jumping_indexes[0]-1].time
		return result
		
	def jumps(self):
		return DataSet([self.filtered_data[index].copy() for index in self.jumping_indexes],timefactor=self.pos_data.timefactor).align_time(self.starting_time())
		
