#!/usr/bin/python3

from .position_dataset import PositionDataSet
from .pose_datapoint import PoseDataPoint

class PoseDataSet(PositionDataSet):
	"""docstring for DataSet."""
	
	@property
	def phi(self):
		return self.get_index(3)
		
	@property
	def theta(self):
		return self.get_index(4)
		
	@property
	def psi(self):
		return self.get_index(5)
		
	def __neg__(self):
		result = PoseDataSet()
		for datapoint in self.data:
			if datapoint is None:
				result.append(None)
			else:
				result.append(-datapoint)
		return result	
		
	def __add__(self, x):
		result = PoseDataSet()
		if isinstance (x, PoseDataSet):
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
		result = PoseDataSet()
		for datapoint in self.data:
			result.append(abs(datapoint))
		return result
		
	def __getitem__(self, index):
		if isinstance(index, list):
			result = PoseDataSet()
			for i in index:
				result.append(self[i])
			return result
		
		result = self.data[index]
		if isinstance(result, list):
			res = PoseDataSet()
			for i in result:
				res.append(i)
			return res
		return result
		
	def copy(self):
		result = PoseDataSet()
		for i in self.data:
			result.append(i.copy())
		return result
		
	# Euler differentiation. This assumes datapoints are in chronological order of time.
	def diff(self):
		result = PoseDataSet()
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
			result.append(PoseDataPoint(time,value))
		if len(result) > 0:
			result.align_time(-(self[1].time - self[0].time) / 2 - self[0].time)
		return result

