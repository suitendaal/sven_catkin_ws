#!/usr/bin/python3

from datalib import *
import numpy as np

class TrajectoryDesigner(object):
	def __init__(self, frequency):
		self.frequency = frequency
		
	def design_trajectory(self, via_points, force_func=None):
		result = PoseDataSet()
		
		if force_func is not None:
			force_result = PoseDataSet()
		
		for i in range(len(via_points)-1):
			t_start = via_points[i].time
			t_end = via_points[i+1].time
			timespan = np.arange(t_start, t_end, 1/self.frequency)
			
			for t in timespan:
				value = []
				for j in range(len(via_points[i].value)):
					value.append(via_points[i].value[j] + (via_points[i+1].value[j] - via_points[i].value[j]) * (t - t_start) / (t_end - t_start))
				result.append(PoseDataPoint(t, value))
				if force_func is not None:
					force_result.append(PoseDataPoint(t, force_func(t, value)))
			
		result.append(via_points[-1].copy())
		if force_func is not None:
			force_result.append(PoseDataPoint(via_points[-1].time, force_func(via_points[-1].time, via_points[-1].value)))
			
		if force_func is not None:
			return result, force_result
		return result
