from .demo_variable import *

class RobotVariable(object):
	def __init__(self, datasets, impact_intervals, filter):
		demo_variables = []
		for i in range(len(datasets)):
			demo_variables.append(DemoVariable(datasets[i], impact_intervals[i], filter))
		
