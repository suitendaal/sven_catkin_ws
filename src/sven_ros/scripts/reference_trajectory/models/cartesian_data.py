#!/usr/bin/python3

from datalib import *

class CartesianData(object):
	"""docstring for CartesianData."""

	def __init__(self, x, y, z, q, jump_intervals, **kwargs):
		self.x = x
		self.y = y
		self.z = z
		self.q = q
		self.jump_intervals = jump_intervals
		
		self.x_filtered = DataSet()
		self.y_filtered = DataSet()
		self.z_filtered = DataSet()
		self.q_filtered = DataSet()
		
		self.pos_info = []
		self.vel_info = []
		
		self.x_diff = x.diff()
		self.y_diff = y.diff()
		self.z_diff = z.diff()
		self.q_diff = q.diff()
		
		self.x_vel_est = DataSet()
		self.y_vel_est = DataSet()
		self.z_vel_est = DataSet()
		self.q_vel_est = DataSet()
		
	def get_start_end(self, phase):
		if phase == 0:
			start = 0
		elif phase > 0:
			start = jump_intervals[phase-1][1] + 1
		else:
			start = -1
			
		if phase == len(jump_intervals) + 1:
			end = -1
		elif phase < len(jump_intervals) + 1:
			end = jump_intervals[phase][0]
		else:
			end = 0
		
		return start, end
		
		
	def get_x(self, phase=-1):
		if phase == -1:
			return self.x
		
		start, end = self.get_start_end(phase)
		return self.x[start:end]
		
	def get_y(self, phase=-1):
		if phase == -1:
			return self.y
		
		start, end = self.get_start_end(phase)
		return self.y[start:end]
		
	def get_z(self, phase=-1):
		if phase == -1:
			return self.z
		
		start, end = self.get_start_end(phase)
		return self.z[start:end]
		
	def get_q(self, phase=-1):
		if phase == -1:
			return self.q
		
		start, end = self.get_start_end(phase)
		return self.q[start:end]
		
	def get_x_filtered(self, phase=-1):
		if phase == -1:
			return self.x_filtered
		
		start, end = self.get_start_end(phase)
		return self.x_filtered[start:end]
		
	def get_y_filtered(self, phase=-1):
		if phase == -1:
			return self.y_filtered
		
		start, end = self.get_start_end(phase)
		return self.y_filtered[start:end]
		
	def get_z_filtered(self, phase=-1):
		if phase == -1:
			return self.z_filtered
		
		start, end = self.get_start_end(phase)
		return self.z_filtered[start:end]
		
	def get_q_filtered(self, phase=-1):
		if phase == -1:
			return self.q_filtered
		
		start, end = self.get_start_end(phase)
		return self.q_filtered[start:end]
		
	def get_x_diff(self, phase=-1):
		if phase == -1:
			return self.x_diff
		
		start, end = self.get_start_end(phase)
		if end >= 0:
			end -= 1
		if start > 0:
			start += 1
		return self.x_diff[start:end]
		
	def get_y_diff(self, phase=-1):
		if phase == -1:
			return self.y_diff
		
		start, end = self.get_start_end(phase)
		if end >= 0:
			end -= 1
		if start > 0:
			start += 1
		return self.y_diff[start:end]
		
	def get_z_diff(self, phase=-1):
		if phase == -1:
			return self.z_diff
		
		start, end = self.get_start_end(phase)
		if end >= 0:
			end -= 1
		if start > 0:
			start += 1
		return self.z_diff[start:end]
		
	def get_q_diff(self, phase=-1):
		if phase == -1:
			return self.q_diff
		
		start, end = self.get_start_end(phase)
		if end >= 0:
			end -= 1
		if start > 0:
			start += 1
		return self.q_diff[start:end]
		
	def get_x_vel(self, phase=-1):
		if phase == -1:
			return self.x_vel_est
		
		start, end = self.get_start_end(phase)
		return self.x_vel_est[start:end]
		
	def get_y_vel(self, phase=-1):
		if phase == -1:
			return self.y_vel_est
		
		start, end = self.get_start_end(phase)
		return self.y_vel_est[start:end]
		
	def get_z_vel(self, phase=-1):
		if phase == -1:
			return self.z_vel_est
		
		start, end = self.get_start_end(phase)
		return self.z_vel_est[start:end]
		
	def get_q_vel(self, phase=-1):
		if phase == -1:
			return self.q_vel_est
		
		start, end = self.get_start_end(phase)
		return self.q_vel_est[start:end]
		
