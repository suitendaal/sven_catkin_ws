#!/usr/bin/python3

from .position_datapoint import *
import numpy as np

class PoseDataPoint(DataPoint):
	"""docstring for PoseDataPoint."""
		
	@property
	def phi(self):
		if self.value is not None:
			return self.value[3]
		return None
		
	@phi.setter
	def phi(self, value):
		if self.value is None:
			self._init_value()
		self.value[3] = value
		
	@property
	def theta(self):
		if self.value is not None:
			return self.value[4]
		return None
		
	@theta.setter
	def theta(self, value):
		if self.value is None:
			self._init_value()
		self.value[4] = value
		
	@property
	def psi(self):
		if self.value is not None:
			return self.value[5]
		return None
		
	@psi.setter
	def psi(self, value):
		if self.value is None:
			self._init_value()
		self.value[5] = value
		
	def _init_value(self):
		self.value = []
		for i in range(6):
			self.value.append(None)

	def __neg__(self):
		if self.value is None:
			result = PoseDataPoint(self.time, None)
		else:
			x = None if self.x is None else -self.x
			y = None if self.y is None else -self.y
			z = None if self.z is None else -self.z
			phi = None if self.phi is None else -self.phi
			theta = None if self.theta is None else -self.theta
			psi = None if self.psi is None else -self.psi
			result = PoseDataPoint(self.time, [x,y,z,phi,theta,psi])
		return result

	def __add__(self, value):
		if value is None:
			return None
		result = PoseDataPoint(self.time, self.value)
		if self.value is not None:
			if isinstance(value, PoseDataPoint) or (isinstance(value, DataPoint) and value.value is not None and len(value.value) == 6):
				if value.value is None:
					result.value = None
				else:
					result.value = []
					for i in range(6):
						result.value.append(None if (self.value[i] is None or value.value[i] is None) else self.value[i] + value.value[i])
			else:
				result.value = []
				for i in range(6):
					result.value = None if self.value[i] is None else self.value[i] + value
		return result

	def __eq__(self, x):
		if isinstance (x, PoseDataPoint) or isinstance(x, DataPoint):
			return self.value == x.value
		else:
			return self.value == x

	def __getitem__(self, index):
		result = DataPoint(self.time, self.value[index])
		return result
		
	def __abs__(self):
		if self.value is None:
			result = PoseDataPoint(self.time, None)
		else:
			result = PoseDataPoint(self.time, None)
			result.x = None if self.x is None else abs(self.x)
			result.y = None if self.y is None else abs(self.y)
			result.z = None if self.z is None else abs(self.z)
			result.phi = None if self.phi is None else abs(self.phi)
			result.theta = None if self.theta is None else abs(self.theta)
			result.psi = None if self.psi is None else abs(self.psi)
		return result
		
	def copy(self):
		result = PoseDataPoint(self.time, self.value)
		return result

