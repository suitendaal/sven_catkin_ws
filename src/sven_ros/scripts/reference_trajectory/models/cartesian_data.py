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
		
		self.x_filtered = DataSet(timefactor=self.x.timefactor)
		self.y_filtered = DataSet(timefactor=self.y.timefactor)
		self.z_filtered = DataSet(timefactor=self.z.timefactor)
		self.q_filtered = []
		for i in range(len(self.q)):
			self.q_filtered.append(DataSet(timefactor=self.q[i].timefactor))
		
		self.pos_info = []
		self.vel_info = []
		self.or_info = []
		
		self.x_diff = x.diff()
		self.y_diff = y.diff()
		self.z_diff = z.diff()
		
		self.x_vel_est = DataSet(timefactor=self.x.timefactor)
		self.y_vel_est = DataSet(timefactor=self.y.timefactor)
		self.z_vel_est = DataSet(timefactor=self.z.timefactor)
		
	def filter(self, **kwargs):
		position_filter = kwargs.get('position_filter',None)
		velocity_estimator = kwargs.get('velocity_estimator',None)
		orientation_filter = kwargs.get('orientation_filter',None)
		
		if position_filter is None:
			self.x_filtered = self.x.copy()
			self.y_filtered = self.y.copy()
			self.z_filtered = self.z.copy()
		else:
			phases = len(self.jump_intervals) + 1
			for i in range(phases):
				start, end = self.get_start_end(i)
				if end == -1:
					end = len(self.x)
				for j in range(start,end):
					x_filtered = position_filter.filter(self.x[start:(j+1)])[0][-1]
					x_filtered.time = self.x[j].time
					self.x_filtered.append(x_filtered, reset_time=False)
					y_filtered = position_filter.filter(self.y[start:(j+1)])[0][-1]
					y_filtered.time = self.y[j].time
					self.y_filtered.append(y_filtered, reset_time=False)
					z_filtered = position_filter.filter(self.z[start:(j+1)])[0][-1]
					z_filtered.time = self.z[j].time
					self.z_filtered.append(z_filtered, reset_time=False)
				if i != phases - 1:
					start, end = self.jump_intervals[i]
					if end == -1:
						end = len(self.x)
					for j in range(start, end):
						self.x_filtered.append(self.x[j].copy(), reset_time=False)
						self.y_filtered.append(self.y[j].copy(), reset_time=False)
						self.z_filtered.append(self.z[j].copy(), reset_time=False)
		
		if velocity_estimator is None:
			self.x_vel_est = self.x_diff.copy()
			self.y_vel_est = self.y_diff.copy()
			self.z_vel_est = self.z_diff.copy()
		else:
			phases = len(self.jump_intervals) + 1
			for i in range(phases):
				start, end = self.get_start_end(i)
				if end == -1:
					end = len(self.x)
				for j in range(start,end):
					x_vel = velocity_estimator.estimate(self.x[start:(j+1)])[0][-1]
					x_vel.time = self.x[j].time
					self.x_vel_est.append(x_vel, reset_time=False)
					y_vel = velocity_estimator.estimate(self.y[start:(j+1)])[0][-1]
					y_vel.time = self.y[j].time
					self.y_vel_est.append(y_vel, reset_time=False)
					z_vel = velocity_estimator.estimate(self.z[start:(j+1)])[0][-1]
					z_vel.time = self.z[j].time
					self.z_vel_est.append(z_vel, reset_time=False)
				if i != phases - 1:
					start, end = self.jump_intervals[i]
					if end == -1:
						end = len(self.x)
					for j in range(start, end):
						x_vel = self.x[j].copy()
						x_vel.value = None
						self.x_filtered.append(x_vel, reset_time=False)
						y_vel = self.y[j].copy()
						y_vel.value = None
						self.y_filtered.append(y_vel, reset_time=False)
						z_vel = self.z[j].copy()
						z_vel.value = None
						self.z_filtered.append(z_vel, reset_time=False)
			
		if orientation_filter is None:
			for i in range(len(self.q)):
				self.q_filtered[i] = self.q[i].copy()
		else:
			phases = len(self.jump_intervals) + 1
			for i in range(phases):
				start, end = self.get_start_end(i)
				if end == -1:
					end = len(self.x)
				for j in range(start,end):
					for k in range(len(self.q)):
						q_filtered = orientation_filter.filter(self.q[k][start:(j+1)])[0][-1]
						q_filtered.time = self.q[k][j].time
						self.q_filtered[k].append(q_filtered, reset_time=False)
				if i != phases - 1:
					start, end = self.jump_intervals[i]
					if end == -1:
						end = len(self.x)
					for j in range(start, end):
						for k in range(len(self.q)):
							self.q_filtered[k].append(self.q[k][j].copy(), reset_time=False)
		
	def get_start_end(self, phase):
		if phase == 0:
			start = 0
		elif phase > 0:
			start = self.jump_intervals[phase-1][1] + 1
		else:
			start = -1
			
		if phase == len(self.jump_intervals):
			end = -1
		elif phase < len(self.jump_intervals):
			end = self.jump_intervals[phase][0]
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
		
