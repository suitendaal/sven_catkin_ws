#!/usr/bin/python3

from .datapoint import DataPoint

class DataSet():
	"""docstring for DataSet."""

	def __init__(self, *args, timefactor=1):
		self.data = list(*args)
		self.timefactor = timefactor
		self.starting_time = 0
		self.aligned = False
		for item in self.data:
			item.time = (item.timestamp - (data[0].timestamp if self.aligned else 0)) / self.timefactor + self.starting_time
			
	def __neg__(self):
		result = DataSet()
		for datapoint in self.data:
			if datapoint is None:
				result.append(None)
			else:
				result.append(-datapoint, reset_time=False)
		return result	
		
	def __len__(self):
		return len(self.data)

	def __add__(self, x):
		result = DataSet()
		if isinstance (x, DataSet):
			if len(x) == len(self):
				for i in range(len(self)):
					if self[i] is None:
						result.append(None)
					else:
						result.append(self[i] + x[i], reset_time=False)
			else:
				return None
		else:
			for datapoint in self.data:
				if datapoint is None:
					result.append(None)
				else:
					result.append(datapoint + x, reset_time=False)
		return result

	def __sub__(self, x):
		return self.__add__(-x)
		
	def __abs__(self):
		result = DataSet()
		for datapoint in self.data:
			result.append(abs(datapoint), reset_time=False)
		return result
		
	def __getitem__(self, index):
		result = self.data[index]
		if isinstance(result, list):
			res = DataSet(timefactor=self.timefactor)
			res.aligned = self.aligned
			res.starting_time = self.starting_time
			for i in result:
				res.append(i, reset_time=False)
			return res
		return result

	def __setitem__(self, index, item):
		item.time = (item.timestamp - (self.data.__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor + self.starting_time
		return self.data.__setitem__(index, item)
		
	def __str__(self):
		return self.data.__str__()
		
	def copy(self):
		result = DataSet(timefactor=self.timefactor)
		result.starting_time = self.starting_time
		result.aligned = self.aligned
		for i in self.data:
			result.append(i.copy(), reset_time=False)
		return result
		
	def append(self, item, reset_time=False):
		result = self.data.append(item)
		if reset_time and item is not None:
			item.time = (item.timestamp - (self.data.__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor + self.starting_time
		return result

	def align_time(self, starting_time=0):
		self.starting_time = starting_time
		self.aligned = True
		for item in self.data:
			item.time = (item.timestamp - (self.data.__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor + self.starting_time
		return self

	def get_xy(self):
		return self.time(), self.values()
		
	def time(self):
		x = []
		for datapoint in self.data:
			x.append(datapoint.time)
		return x
	
	def values(self):
		y = []
		for datapoint in self.data:
			y.append(datapoint.value)
		return y
		
	# Euler differentiation. This assumes datapoints are in chronological order of time.
	def diff(self):
		result = DataSet(timefactor=self.timefactor)
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
			timestamp = self[i-1].timestamp + (self[i].timestamp - self[i-1].timestamp) / 2
			result.append(DataPoint(timestamp,value))
		if len(result) > 0:
			result.align_time(-(self[1].time - self[0].time) / 2 - self[0].time)
		return result
		
	# If value is a list
	def get_index(self, index):
		result = DataSet(timefactor=self.timefactor)
		for i in self.data:
			result.append(i[index], reset_time=False)
		return result
		
	def pop(self, index):
		return self.data.pop(index)


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
