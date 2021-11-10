#!/usr/bin/python3

from .datasets_handle import *

class RobotStateDataSetsHandle(DataSetsHandle):
	def __init__(self, position_datasets, velocity_datasets, orientation_datasets, force_datasets, torque_datasets, impact_intervals):
		super(RobotStateDataSetsHandle, self).__init__(position_datasets, velocity_datasets, orientation_datasets, impact_intervals)
		self.force_trajectory_handles = []
		self.torque_trajectory_handles = []
		
		for i in range(len(force_datasets)):
			self.force_trajectory_handles.append(TrajectoryHandle([force_datasets[i]], impact_intervals[i]))
			self.torque_trajectory_handles.append(TrajectoryHandle([torque_datasets[i]], impact_intervals[i]))
			
	def extend_force_data(self, extender):
		self.extend_data(self.force_trajectory_handles, extender)
		
	def extend_torque_data(self, extender):
		self.extend_data(self.torque_trajectory_handles, extender)
		
	def align_force_data(self, trim=False):
		self.align_phase_times(self.force_trajectory_handles, trim)
		
	def align_torque_data(self, trim=False):
		self.align_phase_times(self.torque_trajectory_handles, trim)
		
	def create_force_promps(self, rbf_width, rbfs_per_second):
		return self.create_promps(self.force_trajectory_handles, rbf_width, rbfs_per_second)
		
	def create_torque_promps(self, rbf_width, rbfs_per_second):
		return self.create_promps(self.torque_trajectory_handles, rbf_width, rbfs_per_second)
	
