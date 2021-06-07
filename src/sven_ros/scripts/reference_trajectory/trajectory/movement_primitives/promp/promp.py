#!/usr/bin/python3

from datalib import *

class ProMP(object):
	"""docstring for ProMP."""
	
	def __init__(self, basis_functions, **kwargs):
		self.basis_functions = basis_functions
		self.weights = []
		
	def learn(self, datasets):
		mu = mean(datasets)
		
	def mean(self, datasets):
		result = DataSet()
		for i in datasets:
			result.append(i.mean())
		return result
		
