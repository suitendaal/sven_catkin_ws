#!/usr/bin/python3

from scipy.spatial.transform import Rotation
from datalib import *

class RobotPoseEvaluator(object):
	def __init__(self, rotation_matrix, position_promp_handles, orientation_promp_handles):
		self.rotation_matrix = rotation_matrix
		self.position_promp_handles = position_promp_handles
		self.orientation_promp_handles = orientation_promp_handles
		
	@property
	def x_promp(self):
		return self.position_promp_handles[0]
		
	@property
	def y_promp(self):
		return self.position_promp_handles[1]
		
	@property
	def z_promp(self):
		return self.position_promp_handles[2]
		
	@property
	def or_x_promp(self):
		return self.orientation_promp_handles[0]
		
	@property
	def or_y_promp(self):
		return self.orientation_promp_handles[1]
		
	@property
	def or_z_promp(self):
		return self.orientation_promp_handles[2]
		
	@property
	def extended_starting_time(self):
		t_start, t_end = self.x_promp.get_extended_start_end()
		return t_start
		
	@property
	def extended_ending_time(self):
		t_start, t_end = self.x_promp.get_extended_start_end()
		return t_end
		
	def evaluate(self, time, via_points=None):
		position_via_points, orientation_via_points = None, None
		
		if via_points is not None:
			if len(via_points) >= 3:
				position_viapoints = via_points[0:3]
			if len(via_points) == 6:
				orientation_viapoints = via_points[3:6]
	
		position = []
		velocity = []
		for i in range(3):
			promp_via_points = DataSet()
			if position_via_points is not None:
				promp_via_points = position_via_points[i]
			position.append(self.position_promp_handles[i].evaluate(time, via_points=promp_via_points)[0][0])
			velocity.append(self.position_promp_handles[i].evaluate(time, derivative=1, via_points=promp_via_points)[0][0])
		
		orientation = []
		if orientation_via_points is not None:
			normalized_orientation_via_points = []
			for i in range(3):
				normalized_orientation_via_points.append(DataSet())
			for i in range(len(orientation_via_points[0])):
				orientation_values = []
				for j in range(3):
					orientation_values.append(orientation_via_points[j][i].value)
				rotation = Rotation.from_euler('xyz',orientation_values)
				new_orientation_values = Rotation.from_matrix(self.rotation_matrix.dot(rotation.as_matrix())).as_euler('xyz')
				for j in range(3):
					normalized_orientation_via_points[j].append(DataPoint(orientation_via_points[j][i].time, new_orientation_values[j]))
		for i in range(3):
			promp_via_points = DataSet()
			if orientation_via_points is not None:
				promp_via_points = normalized_orientation_via_points[i]
			orientation.append(self.orientation_promp_handles[i].evaluate(time, via_points=promp_via_points)[0][0])
		normalized_orientation = Rotation.from_matrix(self.rotation_matrix.T.dot(Rotation.from_euler('xyz',orientation).as_matrix())).as_euler('xyz').tolist()
		return position, velocity, normalized_orientation

