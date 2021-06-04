#!/usr/bin/python3

from .datapoint import DataPoint

class DataSet(list):
	"""docstring for DataSet."""

	def __init__(self, *args, timefactor=1):
		list.__init__(self, *args)
		self.timefactor = timefactor
		self.starting_time = 0
		self.aligned = False
		for item in self:
			item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time		

	def __add__(self, x):
		result = DataSet()
		for datapoint in self:
			if datapoint is None:
				result.append(None)
			else:
				result.append(datapoint + x)
		return result

	def __sub__(self, x):
		return self.__add__(-x)

	def __setitem__(self, index, item):
		item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time
		return super().__setitem__(index, item)

	def align_time(self, starting_time=0):
		self.starting_time = starting_time
		self.aligned = True
		for item in self:
			item.time = (item.timestamp - (super().__getitem__(0).timestamp if self.aligned else 0)) / self.timefactor - self.starting_time

	def get_xy(self):
		x = []
		y = []
		for datapoint in self:
			x.append(datapoint.time)
			y.append(datapoint.value)
		return x, y


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
