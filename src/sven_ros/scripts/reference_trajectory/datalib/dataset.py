#!/usr/bin/python3

from .datapoint import DataPoint

class DataSet(list):
	"""docstring for DataSet."""

	def __init__(self, *args, timefactor=1):
		super(DataSet, self).__init__(*args)
		self.timefactor = timefactor
		self.starting_time = 0
		self.aligned = False
		for item in self:
			item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time	
			
	def __neg__(self):
		result = DataSet()
		for datapoint in self:
			if datapoint is None:
				result.append(None)
			else:
				result.append(-datapoint)
		return result	

	def __add__(self, x):
		result = DataSet()
		if isinstance (x, DataSet):
			if len(x) == len(self):
				for i in range(len(self)):
					if self[i] is None:
						result.append(None)
					else:
						result.append(self[i] + x[i])
			else:
				return None
		else:
			for datapoint in self:
				if datapoint is None:
					result.append(None)
				else:
					result.append(datapoint + x)
		return result

	def __sub__(self, x):
		return self.__add__(-x)
		
	def __abs__(self):
		result = DataSet()
		for datapoint in self:
			result.append(abs(datapoint))
		return result

	def __setitem__(self, index, item):
		item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time
		return super().__setitem__(index, item)
		
	def copy(self):
		result = DataSet()
		for i in self:
			result.append(i.copy())

	def align_time(self, starting_time=0):
		self.starting_time = starting_time
		self.aligned = True
		for item in self:
			item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time
		return self

	def get_xy(self):
		return self.time(), self.values()
		
	def time(self):
		x = []
		for datapoint in self:
			x.append(datapoint.time)
		return x
	
	def values(self):
		y = []
		for datapoint in self:
			y.append(datapoint.value)
		return y
		
	# Euler differentiation. This assumes datapoints are in chronological order of time.
	def diff(self):
		result = DataSet(timefactor=self.timefactor)
		for i in range(1,len(self)):
			if self[i].value is not None and self[i-1].value is not None:
				value = (self[i].value - self[i-1].value) / (self[i].time - self[i-1].time)
			else:
				value = None
			timestamp = self[i-1].timestamp + (self[i].timestamp - self[i-1].timestamp) / 2
			result.append(DataPoint(timestamp,value))
		if len(result) > 1:
			result.align_time(-(self[1].time - self[0].time) / 2)
		return result


def main():
    data = DataSet()
    data.append(DataPoint(4,1))
    data.append(DataPoint(5,2))
    data.append(DataPoint(6,3))
    print(data)
    print(data[2])
    data[2] = DataPoint(6,4)
    print(data[2])
    data.align_time()
    print(data)

if __name__ == '__main__':
    main()
