from datalib import *

class FilteredDataSet(object):
	def __init__(self, data, filter):
		self.data = data
		self.filter_ = filter
		self.filtered_data = self.filter()
		
	def filter(self):
		result = DataSet()
		self.filter_.reset()
		for i in range(len(self.data)):
			filtered_datapoint, coefs = self.filter_.update(self.data[i])
			result.append(filtered_datapoint)
		return result
		
