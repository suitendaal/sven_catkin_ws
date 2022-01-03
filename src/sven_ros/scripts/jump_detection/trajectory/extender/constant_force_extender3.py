#!/usr/bin/python3

from .extender import *
import numpy as np
import math
import scipy.optimize as optimization

class ConstantForceExtender3(Extender):

	def __init__(self, frequency, impact_duration=0, timespan=0, bounds=None):
		super().__init__(frequency, impact_duration, timespan)
		self.bounds = bounds
		if self.bounds is None:
			omega = 2*math.pi / (self.impact_duration/3)
			self.bounds = ((-np.inf, 0, -np.log(10)/((3/2)*self.impact_duration/3), -np.pi, (2/3)*omega), (np.inf, np.inf, 0, np.pi, (3.2)*omega))

	def calc_post_impact_force(self, phase, trajectory_handle):
		index = self.post_impact_starting_index(phase, trajectory_handle)
		datapoint = trajectory_handle.trajectory_data[index]
		return PositionDataPoint(datapoint.time, datapoint.value[0])

	def extend_before(self, phase, trajectory_handle, trajectory):
		times = self.get_times_before(trajectory)
		result = DataSet()
		
#		starting_time = trajectory[0].time
		post_impact_force = self.calc_post_impact_force(phase, trajectory_handle)
		
		for t in times:
			value = post_impact_force.value				
			result.append(DataPoint(t, [value]))
		
		for datapoint in trajectory:
			result.append(datapoint)
		
		return result
		
	def extend_after(self, phase, trajectory_handle, trajectory):
		times = self.get_times_after(trajectory)
		
		starting_time = trajectory[-1].time
		force_value = trajectory[-1].value[0]
		
		for t in times:
			value = force_value.copy()
			trajectory.append(DataPoint(t, [value]))
		
		return trajectory
	
