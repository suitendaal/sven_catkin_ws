#!/usr/bin/python3

import random
import numpy as np

class ForceSimulator(object):
	def __init__(self, f_max=0, tau_max=0, weight_factor=0.9, previous_force=None):
		self.f_max = f_max
		self.tau_max = tau_max
		self.previous_force = previous_force
		self.weight_factor = weight_factor
		
	def simulate_force(self, t, pose):
		f_value = random.uniform(0, self.f_max)
		tau_value = random.uniform(0, self.tau_max)
		if self.previous_force is not None:
			f_value = (1-self.weight_factor) * f_value + self.weight_factor * np.linalg.norm(self.previous_force[0:3])
			tau_value = (1-self.weight_factor) * tau_value + self.weight_factor * np.linalg.norm(self.previous_force[3:6])
		
		f_init = []
		tau_init = []
		for i in range(3):
			f_init.append(random.uniform(-1, 1))
			tau_init.append(random.uniform(-1, 1))
			
		f_direction = normalize_array(f_init)
		tau_direction = normalize_array(tau_init)
		if self.previous_force is not None:
			f_prev_direction = normalize_array(self.previous_force[0:3])
			tau_prev_direction = normalize_array(self.previous_force[3:6])
		else:
			f_prev_direction = [0, 0, 0]
			tau_prev_direction = [0, 0, 0]
			
		for i in range(3):
			f_direction[i] = (1-self.weight_factor) * f_direction[i] + self.weight_factor * f_prev_direction[i]
			tau_direction[i] = (1-self.weight_factor) * tau_direction[i] + self.weight_factor * tau_prev_direction[i]
			
		f_direction = normalize_array(f_direction)
		tau_direction = normalize_array(tau_direction)

		f_result = []
		tau_result = []
		for i in range(3):
			f_result.append(f_value * f_direction[i])
			tau_result.append(tau_value * tau_direction[i])
		result = f_result
		result.extend(tau_result)
		self.previous_force = result
		return result
			
def normalize_array(array):
	mag = np.linalg.norm(array)
	result = []
	for i in range(len(array)):
		if mag > 0:
			result.append(array[i] / mag)
		else:
			result.append(0)
	return result

