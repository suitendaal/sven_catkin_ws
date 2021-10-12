#!/usr/bin/python3

from datalib import *

class Extender(object):
	def __init__(self):
		pass
		
	def extend(self, phase, trajectory_handle):
		result = trajectory_handle.trajectory_data[trajectory_handle.trimmed_post_impact_index(phase):trajectory_handle.trimmed_ante_impact_index(phase)].copy()
		result.align_time(result[0].time + trajectory_handle.phase_time_shifts[phase])
		return result
