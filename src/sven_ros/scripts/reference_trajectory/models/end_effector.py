#!/usr/bin/python3

from .cartesian_data import *

class EndEffector(object):
	"""docstring for Joint."""

	def __init__(self, position_filter=None, velocity_estimator=None, orientation_filter=None, **kwargs):
		self.cartesian_data = []
		self.position_filter = position_filter
		self.velocity_estimator = velocity_estimator
		self.orientation_filter = orientation_filter
		
	def append_data(self, x, y, z, q, jump_intervals):
		self.cartesian_data.append(CartesianData(x,y,z,q,jump_intervals))
		
	def filter(self, datasets=None):
		if datasets is None:
			datasets = range(len(joint_data))
		for i in datasets:
			self.cartesian_data[i].filter(position_filter=self.position_filter, velocity_estimator=self.velocity_estimator, orientation_filter=self.orientation_filter)			
		
