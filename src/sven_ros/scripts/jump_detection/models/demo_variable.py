from datalib import *

class DemoVariable(object):
	def __init__(self, data, impact_intervals, filter):
		self.data = data
		self.impact_intervals = impact_intervals
		self.filtered_data = DataSet()
		self.filter_data(filter)
		
	@property
	def n_phases(self):
		return len(self.impact_intervals) + 1
		
	def get_starting_index(self, phase):
		start = 0
		if phase > 0:
			start = self.impact_intervals[phase-1][-1] + 1
		return start
	
	def get_ending_index(self, phase):
		end = len(position_data[0])
		if phase < len(self.impact_intervals):
			end = self.impact_intervals[phase][0]
		return end
		
	def get_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		return self.data[start:end]
		
	def get_filtered_data(self, phase):
		start = self.get_starting_index(phase)
		end = self.get_ending_index(phase)
		return self.filtered_data[start:end]
		
	def filter_data(self, filter):
		self.filtered_data = DataSet()
		
		for phase in range(self.n_phases):
			start = self.get_starting_index(phase)
			end = self.get_ending_index(phase)
			
			filter.reset()
			for i in range(start, end):
				filtered_datapoint, coefs = filter.update(data[i])
				self.filtered_data.append(filtered_datapoint)
			if phase < self.n_phases - 1:
				for i in range(self.impact_intervals[phase][0], self.impact_intervals[phase][-1] + 1):
					self.filtered_data.append(self.data[i].copy())
		
