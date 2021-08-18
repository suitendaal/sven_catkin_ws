#!/usr/bin/python3

import numpy as np

class RobotState(object):
	def __init__(self, robot_state_msg):
		self.msg_ = robot_state_msg
		
	@property
	def coriolis(self):
		return self.msg_.coriolis
		
	# Check if this is correct
	@property
	def jacobian(self):
		tmp = np.array(self.msg_.jacobian)
		return tmp.reshape(6,7,order='F')
		
	@property
	def q(self):
		return self.msg_.q
		
	@property
	def dq(self):
		return self.msg_.dq
		
	@property
	def pose_matrix(self):
		tmp = np.array(self.msg_.O_T_EE)
		return tmp.reshape(4,4,order='F')
		
	@property
	def position_desired(self):
		return self.msg_.position_d
		
	@property
	def orientation_desired(self):
		return self.msg_.orientation_d
		
	@property
	def tau_J_d
