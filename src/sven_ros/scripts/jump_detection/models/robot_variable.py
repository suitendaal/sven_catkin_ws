from .demo_variable import *

class RobotVariable(object):
	def __init__(self, datasets, derivative_datasets, impact_intervals):
		self.demo_variables = []
		for i in range(len(datasets)):
			demo_variable = DemoVariable(datasets[i], derivative_datasets[i], impact_intervals[i])			
			self.demo_variables.append(demo_variable)
			
	@property
	def n_phases(self):
		if len(self.demo_variables) > 0:
			return self.demo_variables[0].n_phases
		return 0
		
	def filter_data(self, data_filter):
		for variable in self.demo_variables:
			variable.filter_data(data_filter)
	
	def filter_derivative(self, derivative_filter):
		for variable in self.demo_variables:
			variable.filter_derivative(derivative_filter)
			
	def extend_data(self, extender):
	
		# Determine starting time of extended phase
		if len(self.demo_variables) > 0:
			for phase in range(self.demo_variables[0].n_phases):
				t_start = None
				t_end = None
				for variable in self.demo_variables:
					if t_start is None or variable.get_starting_time(phase) + variable.time_shift(phase) > t_start:
						t_start = variable.get_starting_time(phase) + variable.time_shift(phase)
					if t_end is None or variable.get_ending_time(phase) + variable.time_shift(phase) < t_end:
						t_end = variable.get_ending_time(phase) + variable.time_shift(phase)
				for variable in self.demo_variables:
					variable.set_extended_time(phase, t_start, t_end)
	
		for variable in self.demo_variables:
			variable.extend_data(extender)
	
