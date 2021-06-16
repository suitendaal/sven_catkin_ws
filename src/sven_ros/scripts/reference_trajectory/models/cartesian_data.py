#!/usr/bin/python3

from datalib import *

class CartesianData(object):
	"""docstring for Joint."""

	def __init__(self, x, y, z, q, jump_indexes, **kwargs):
		self.x = x
		self.y = y
		self.z = z
		self.q = q
		self.jump_indexes = jump_indexes
		
		self.x_filtered = DataSet()
		self.y_filtered = DataSet()
		self.z_filtered = DataSet()
		self.q_filtered = DataSet()
		
		self.info = []
		
		self.x_diff = x.diff()
		self.y_diff = y.diff()
		self.z_diff = z.diff()
		self.q_diff = q.diff()
		
	
