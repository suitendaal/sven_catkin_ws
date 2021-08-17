#!/usr/bin/python3

from .datapoint import DataPoint

class DataSet():
	"""docstring for DataSet."""

	def __init__(self, *args):
		self.data = list(*args)
			
	def __neg__(self):
		result = DataSet()
		for datapoint in self.data:
			if datapoint is None:
				result.append(None)
			else:
				result.append(-datapoint)
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

	def __sub__(self, x):
		return self.__add__(-x)
		
	def __abs__(self):
		result = DataSet()
		for datapoint in self.data:
			result.append(abs(datapoint))
		return result
		
	def __getitem__(self, index):
		result = self.data[index]
		if isinstance(result, list):
			res = DataSet()
			for i in result:
				res.append(i)
			return res
		return result

	def __setitem__(self, index, item):
		return self.data.__setitem__(index, item)
		
	def __str__(self):
		return self.data.__str__()
		
	def clear(self):
		self.data.clear()
		
	def copy(self):
		result = DataSet()
		for i in self.data:
			result.append(i.copy())
		return result
		
	def append(self, item):
		result = self.data.append(item)
		return result

	def align_time(self, starting_time=0):
		times = []
		for item in self.data:
			time = item.time - self.data.__getitem__(0).time + starting_time
			times.append(time)
		for i in range(len(times)):
			self.data[i].time = times[i]
		return self

	def get_xy(self):
		return self.time, self.value
		
	@property
	def time(self):
		x = []
		for datapoint in self.data:
			x.append(datapoint.time)
		return x
	
	@property
	def value(self):
		y = []
		for datapoint in self.data:
			y.append(datapoint.value)
		return y
		
	# Euler differentiation. This assumes datapoints are in chronological order of time.
	def diff(self):
		result = DataSet()
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
			result.append(DataPoint(time,value))
		if len(result) > 0:
			result.align_time(-(self[1].time - self[0].time) / 2 - self[0].time)
		return result
		
	# If value is a list
	def get_index(self, index):
		result = DataSet()
		for i in self.data:
			result.append(i[index])
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
