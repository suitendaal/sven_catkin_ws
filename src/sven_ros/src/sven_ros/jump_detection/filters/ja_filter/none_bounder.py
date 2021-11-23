#!/usr/bin/python3

from .constant_bounder import *

class NoneBounder(ConstantBounder):
	"""docstring for Bounder."""

	def __init__(self, **kwargs):
		super().__init__(bound=None)
		
	def copy(self):
		return NoneBounder()

