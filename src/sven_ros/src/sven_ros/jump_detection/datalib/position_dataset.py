#!/usr/bin/python3

from .dataset import DataSet
from .position_datapoint import PositionDataPoint

class PositionDataSet(DataSet):
	"""docstring for DataSet."""
	
	@property
	def x(self):
		return self.get_index(0)
		
	@property
	def y(self):
		return self.get_index(1)
		
	@property
	def z(self):
		return self.get_index(2)
		
	@property
	def mag(self):
		result = PositionDataSet()
		for datapoint in self:
			result.append(datapoint.mag)
		return result
		
	@staticmethod
	def from_dataset(dataset):
		result = PositionDataSet()
		for datapoint in dataset:
			result.append(PositionDataPoint.from_datapoint(datapoint))
		return result
		
	def __neg__(self):
		result = PositionDataSet()
		for datapoint in self.data:
			if datapoint is None:
				result.append(None)
			else:
				result.append(-datapoint)
		return result	
		
	def __add__(self, x):
		result = PositionDataSet()
		if isinstance (x, PositionDataSet):
			if len(x) == len(self):
				for i in range(len(self)):
					if self[i] is None:
						result.append(None)
					else:
						result.append(self[i] + x[i])
			else:
				return None
		else:
			for datapoint in self.data:
				if datapoint is None:
					result.append(None)
				else:
					result.append(datapoint + x)
		return result
		
	def __abs__(self):
		result = PositionDataSet()
		for datapoint in self.data:
			result.append(abs(datapoint))
		return result
		
	def __getitem__(self, index):
		if isinstance(index, list):
			result = PositionDataSet()
			for i in index:
				result.append(self[i])
			return result
		
		result = self.data[index]
		if isinstance(result, list):
			res = PositionDataSet()
			for i in result:
				res.append(i)
			return res
		return result
		
	def copy(self):
		result = PositionDataSet()
		for i in self.data:
			result.append(i.copy())
		return result
		
	# Euler differentiation. This assumes datapoints are in chronological order of time.
	@property
	def diff(self):
		result = PositionDataSet()
		for i in range(1,len(self)):
			if self[i].value is not None and self[i-1].value is not None:
				if isinstance(self[i].value, list):
					value = []
					for j in range(len(self[i].value)):
						value.append((self[i].value[j] - self[i-1].value[j]) / (self[i].time - self[i-1].time))
				else:
					value = (self[i].value - self[i-1].value) / (self[i].time - self[i-1].time)
			else:
				value = None
			time = self[i-1].time + (self[i].time - self[i-1].time) / 2
			result.append(PositionDataPoint(time,value))
		if len(result) > 0:
			result.align_time(-(self[1].time - self[0].time) / 2 - self[0].time)
		return result

