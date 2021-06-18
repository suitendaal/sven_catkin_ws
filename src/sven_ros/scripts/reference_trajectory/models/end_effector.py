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
			
	def align_time(self, datasets, phase):
		for j in range(len(datasets)):
			if phase == 0:
				# Align to impact
				datasets[j].align_time(datasets[j][-1].time - datasets[j][0].time)
			elif phase == len(self.cartesian_data[0].jump_intervals):
				# Align to start
				datasets[j].align_time()
			else:
				# Align to center
				datasets[j].align_time((datasets[j][-1].time - datasets[j][0].time) / 2)
			
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
							indexes_to_pop = []
							for k in range(len(dataset)):
								if vel_est[k].value is None:
									indexes_to_pop.insert(0,k)
								else:
									dataset[k].value = [dataset[k].value, vel_est[k].value]
							for k in indexes_to_pop:
								dataset.pop(k)
						datasets.append(dataset)
				
				if self.velocity_estimator is None:		
					promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
				else:
					promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
				self.align_time(datasets, phase)
				promp.learn(datasets)
				self.pos_promps.append(promp)
				
		elif promp_type == 'orientation' or promp_type == 'all':
			for i in range(4):
				datasets = []
				for j in self.cartesian_data:
					if j.filtered:
						dataset = j.get_q_filtered(phase)[i].copy()
						datasets.append(dataset)
				self.align_time(datasets, phase)
				promp = ProMP(rbfs,derivatives=0,weights_covariance=1)
				promp.learn(datasets)
				self.or_promps.append(promp)	
	
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
		
		datasets = []		
		for i in self.cartesian_data:
			start, end = i.get_start_end(phase)
			datasets.append(i.x[start:end])
			
		self.align_time(datasets, phase)
		
		for i in datasets:
			if t_start is None or i[0].time < t_start:
				t_start = i[0].time
			if t_end is None or i[-1].time > t_end:
				t_end = i[-1].time
		
		return t_start, t_end
		
