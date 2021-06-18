#!/usr/bin/python3

from .cartesian_data import *
from trajectory import *
from datalib import *

class EndEffector(object):
	"""docstring for Joint."""

	def __init__(self, position_filter=None, velocity_estimator=None, orientation_filter=None, **kwargs):
		self.cartesian_data = []
		self.position_filter = position_filter
		self.velocity_estimator = velocity_estimator
		self.orientation_filter = orientation_filter
		self.pos_promps = []
		self.or_promps = []
		
	def append_data(self, x, y, z, q, jump_intervals):
		self.cartesian_data.append(CartesianData(x,y,z,q,jump_intervals))
		
	def filter(self, datasets=None):
		if datasets is None:
			datasets = range(len(self.cartesian_data))
		for i in datasets:
			self.cartesian_data[i].filter(position_filter=self.position_filter, velocity_estimator=self.velocity_estimator, orientation_filter=self.orientation_filter)
			
	# Phase is which part of the interval
	def create_promps(self, phase, rbfs, promp_type='all'):
		if promp_type == 'position' or promp_type == 'all':
			for i in range(3):
				datasets = []
				for j in self.cartesian_data:
					if j.filtered:
						if i == 0:
							dataset = j.get_x_filtered(phase).copy()
						elif i == 1:
							dataset = j.get_y_filtered(phase).copy()
						else:
							dataset = j.get_z_filtered(phase).copy()
						if self.velocity_estimator is not None:
							if i == 0:
								vel_est = j.get_x_vel(phase).copy()
							elif i == 1:
								vel_est = j.get_y_vel(phase).copy()
							else:
								vel_est = j.get_z_vel(phase).copy()
							for k in range(len(dataset)):
								dataset[k].value = [dataset[k].value, vel_est[k].value]
						datasets.append(dataset)
				
				if self.velocity_estimator is None:		
					self.pos_promps.append(ProMP(rbfs,derivatives=0,weights_covariance=1))
				else:
					self.pos_promps.append(ProMP(rbfs,derivatives=1,weights_covariance=1))
		elif promp_type == 'orientation' or promp_type == 'all':
			for i in range(4):
				datasets = []
				for j in self.cartesian_data:
					if j.filtered:
						dataset = j.get_q_filtered(phase)[i].copy()
						datasets.append(dataset)
				self.or_promps.append(ProMP(rbfs,derivatives=0,weights_covariance=1))	
	
	def create_basis_functions(self, phase, width, rbfs_per_second):
		
		# Get starting and ending time
		t_start, t_end = self.get_time_range(phase)
		
		# Create basis functions
		n_rbfs = round(rbfs_per_second * (t_end - t_start) + 1)
		rbfs = []
		for i in range(n_rbfs):
			center = (t_end - t_start) * i / (n_rbfs-1) + t_start
			rbf = RadialBasisFunction(center=center, width=width)
			rbfs.append(rbf)
		
		return rbfs
		
	def get_time_range(self, phase):
		t_start = None
		t_end = None
		for i in self.cartesian_data:
			start, end = i.get_start_end(phase)
			print(i.x[start].time,i.x[end].time)
			if t_start is None or i.x[start].time < t_start:
				t_start = i.x[start].time
			if t_end is None or i.x[end].time > t_end:
				t_end = i.x[end].time
		return t_start, t_end
		
