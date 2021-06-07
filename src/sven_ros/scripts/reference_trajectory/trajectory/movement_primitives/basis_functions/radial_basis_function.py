#!/usr/bin/python3

import math
from .basis_function import *

class RadialBasisFunction(BasisFunction):
	"""docstring for BasisFunction."""
	
	def __init__(self, **kwargs):
		super(RadialBasisFunction, self).__init__(**kwargs)
		self.center = kwargs.get('center',0)
		self.width = kwargs.get('width',1)
		self.scale_factor = kwargs.get('scale_factor',1)

	def evaluate(self, time, **kwargs):
		return math.exp( -0.5 * ((time - self.center)**2) / self.width) * self.scale_factor

