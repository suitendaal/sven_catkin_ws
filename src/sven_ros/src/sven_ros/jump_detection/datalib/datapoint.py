#!/usr/bin/python3

class DataPoint(object):
	"""docstring for DataPoint."""

	def __init__(self, time, value):
		super(DataPoint, self).__init__()
		self.time = time
		self.value = value

	def __neg__(self):
		if self.value is None:
			result = DataPoint(self.time, None)
		else:
			result = DataPoint(self.time, -self.value)
		return result

	def __add__(self, x):
		if x is None:
			return None
		result = DataPoint(self.time, self.value)
		if self.value is not None:
			if isinstance (x, DataPoint):
				if x.value is None:
					result.value = None
				elif isinstance (self.value, list) and isinstance (x.value, list) and len(self.value) == len(x.value):
					result.value = []
					for i in range(len(x.value)):
						result.value.append(self.value[i] + x.value[i])
				else:
					result.value = self.value + x.value
			else:
				result.value = self.value + x
		return result

	def __sub__(self, x):
		return self.__add__(-x)

	def __eq__(self, x):
		if isinstance (x, DataPoint):
			return self.value == x.value
		else:
			return self.value == x

	def __ne__(self, x):
		return not (self == x)

	def __lt__(self, x):
		if isinstance (x, DataPoint):
			return self.value < x.value
		else:
			return self.value < x

	def __le__(self, x):
		if self < x:
			return True
		return self == x

	def __gt__(self, x):
		return not self <= x

	def __ge__(self, x):
		return not self < x

	def __getitem__(self, index):
		result = DataPoint(self.time, self.value[index])
		return result

	def __setitem__(self, index, value):
		self.value[index] = value

	def __str__(self):
		return "(Time: {}, Value: {})".format(self.time, self.value)

	def __repr__(self):
		return str(self)
		
	def __abs__(self):
		if self.value is None:
			result = DataPoint(self.time, None)
		else:
			result = DataPoint(self.time, abs(self.value))
		return result
		
	def copy(self):
		value = self.value
		if isinstance(value, list):
			value = value.copy()
		result = DataPoint(self.time, value)
		return result
		
	def mean(self):
		result = DataPoint(self.time, mean(self.value))
		return result

def main():
	a = DataPoint(0,4)
	b = DataPoint(1,5)
	c = a + b
	d = b + a
	e = DataPoint(3,"hoi")
	f = DataPoint(4,"hey")
	g = e + f
	print(a)
	print(b)
	print(c)
	print(d)
	print(e)
	print(f)
	print(g)

if __name__ == '__main__':
	main()
