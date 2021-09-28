#!/usr/bin/python3

from .datapoint import *
import numpy as np

class PositionDataPoint(DataPoint):
	"""docstring for PositionDataPoint."""
		
	@property
	def x(self):
		if self.value is not None:
			return self.value[0]
		return None
		
	@x.setter
	def x(self, value):
		if self.value is None:
			self._init_value()
		self.value[0] = value
		
	@property
	def y(self):
		if self.value is not None:
			return self.value[1]
		return None
		
	@y.setter
	def y(self, value):
		if self.value is None:
			self._init_value()
		self.value[1] = value
		
	@property
	def z(self):
		if self.value is not None:
			return self.value[2]
		return None
		
	@z.setter
	def z(self, value):
		if self.value is None:
			self._init_value()
		self.value[2] = value
		
	def _init_value(self):
		self.value = []
		for i in range(3):
			self.value.append(None)

	def __neg__(self):
		if self.value is None:
			result = PositionDataPoint(self.time, None)
		else:
			x = None if self.x is None else -self.x
			y = None if self.y is None else -self.y
			z = None if self.z is None else -self.z
			result = PositionDataPoint(self.time, [x,y,z])
		return result

	def __add__(self, value):
		if value is None:
			return None
		result = PositionDataPoint(self.time, self.value)
		if self.value is not None:
			if isinstance(value, PositionDataPoint) or (isinstance(value, DataPoint) and value.value is not None and len(value.value) == 3):
				if value.value is None:
					result.value = None
				else:
					result.value = []
					for i in range(3):
						result.value.append(None if (self.value[i] is None or value.value[i] is None) else self.value[i] + value.value[i])
			else:
				result.value = []
				for i in range(3):
					result.value = None if self.value[i] is None else self.value[i] + value
		return result

	def __eq__(self, x):
		if isinstance (x, PositionDataPoint) or isinstance(x, DataPoint):
			return self.value == x.value
		else:
			return self.value == x

	def __getitem__(self, index):
		result = DataPoint(self.time, self.value[index])
		return result
		
	def __abs__(self):
		if self.value is None:
			result = PositionDataPoint(self.time, None)
		else:
			result = PositionDataPoint(self.time, None)
			result.x = None if self.x is None else abs(self.x)
			result.y = None if self.y is None else abs(self.y)
			result.z = None if self.z is None else abs(self.z)
		return result
		
	def copy(self):
		result = PositionDataPoint(self.time, self.value)
		return result

