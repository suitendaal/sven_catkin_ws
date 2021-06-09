#!/usr/bin/python3

import math
from .basis_function import *

class RadialBasisFunction(BasisFunction):
	"""docstring for BasisFunction."""
	
	def __init__(self, **kwargs):
		super(RadialBasisFunction, self).__init__(**kwargs)
		self.center = kwargs.get('center',0)
		self.width = kwargs.get('width',1)

	def evaluate(self, time, derivative=0, **kwargs):
		if derivative == 0:
			return math.exp( -0.5 * ((time - self.center)**2) / self.width)
		elif derivative == 1:
			return -(time - self.center) / self.width * self.evaluate(time, derivative=0, **kwargs)
		elif derivative == 2:
			return -(time - self.center) / self.width * self.evaluate(time, derivative=1, **kwargs) + -1 / self.width * self.evaluate(time, derivative=0, **kwargs)
		elif derivative == 3:
			return -(time - self.center) / self.width * self.evaluate(time, derivative=2, **kwargs) + -1 / self.width * self.evaluate(time, derivative=1, **kwargs) + -1 / self.width * self.evaluate(time, derivative=1, **kwargs)
		else:
			return 0

