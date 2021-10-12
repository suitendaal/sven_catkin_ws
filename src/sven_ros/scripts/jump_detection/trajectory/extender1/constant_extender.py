#!/usr/bin/python3

import numpy as np
from datalib import *
from .constant_velocity_extender import *

class ConstantExtender(ConstantVelocityExtender):
	"""docstring for TrajectoryExtender."""
	
	def __init__(self, **kwargs):
		super(ConstantExtender, self).__init__(**kwargs)
		
	def copy(self):
		return ConstantExtender(timesteps=self.timesteps, delta_time=self.delta_time)

	def extend(self, trajectory, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[], **kwargs):
		return super().extend(trajectory, extend_before=extend_before, extend_after=extend_after, extended_times_before=extended_times_before, extended_times_after=extended_times_after)
		
	def extend_velocity(self, velocity_data, extend_before=False, extend_after=False, extended_times_before=[], extended_times_after=[], **kwargs):
		return super().extend_velocity(velocity_data, extend_before=extend_before, extend_after=extend_after, extended_times_before=extended_times_before, extended_times_after=extended_times_after)
	
