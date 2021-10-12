#!/usr/bin/python3

class PostImpactVelocityCalculator(object):
	def __init__(self):
		pass
		
	def calculate_velocity(self, data, derivative_data):
		if derivative_data is None or len(derivative_data) == 0:
			return 0
		return derivative_data[0]
