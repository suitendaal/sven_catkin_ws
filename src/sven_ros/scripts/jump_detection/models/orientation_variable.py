from .robot_variable import *
from .promp_handler import *
from trajectory import *
import numpy as np

class OrientationVariable():
	def __init__(self, orientation_datasets, orientation_derivative_datasets, impact_intervals, rotation_matrix=np.eye(3)):
		self.rotation_matrix = rotation_matrix
		self.robot_variables = []
		for i in range(3):
			self.robot_variables.append([])
		
		for i in range(len(orientation_datasets)):
			normalized_datasets = []
			for j in range(3):
				normalized_datasets.append(DataSet())
			## TODO: do something with the 3 angles of orientation_datasets[i] and the rotation matrix
			for j in range(3):
				normalized_datasets[j] = orientation_datasets[i][j]
			##
			for j in range(3):
				self.robot_variables[j].append(RobotVariable(normalized_datasets[j], orientation_derivative_datasets[j], impact_intervals[i])
		normalized_datasets = []
		for i in range(len(datasets)):
			normalized_dataset = DataSet()
			## TODO: do something with datasets and rotation matrix
			normalized_dataset = datasets[i]
			##
			normalized_datasets.append(normalized_dataset)
		super(OrientationVariable, self).__init__(normalized_datasets, derivative_datasets, impact_intervals)
		
	def create_phase_promp(self, phase, rbfs):
		mp_handler = super(OrientationVariable, self).create_phase_promp(phase, rbfs)
		print("hoi create phase promp")
		## TODO: create custom orientation mp handler with rotation matrix
		return mp_handler				
	
