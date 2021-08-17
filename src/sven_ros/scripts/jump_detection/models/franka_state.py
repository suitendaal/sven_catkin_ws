#!/usr/bin/python3

import numpy as np
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb

class FrankaState(object):
	
	def __init__(self, franka_state_msg):
		self.franka_state_ = franka_state_msg
		self.robot_ = rtb.models.DH.Panda()
	
	@property
	def q(self):
		return self.franka_state_.q
		
	@property
	def q_desired(self):
		return self.franka_state_.q_d
		
	@property
	def dq(self):
		return self.franka_state_.dq
		
	@property
	def dq_desired(self):
		return self.franka_state_.dq_d
		
	@property
	def pose_matrix(self):
		tmp = np.array(self.franka_state_.O_T_EE)
		return tmp.reshape(4,4,order='F')
		
	@property
	def pose_matrix_desired(self):
		tmp = np.array(self.franka_state_.O_T_EE_d)
		return tmp.reshape(4,4,order='F')
		
	@property
	def position(self):
		mat = self.pose_matrix
		result = []
		for i in range(3):
			result.append(mat[i,3])
		return result
		
	@property
	def position_desired(self):
		mat = self.pose_matrix_desired
		result = []
		for i in range(3):
			result.append(mat[i,3])
		return result
		
	@property
	def x(self):
		return self.position[0]
		
	@property
	def x_desired(self):
		return self.position_desired[0]
	
	@property
	def y(self):
		return self.position[1]
		
	@property
	def y_desired(self):
		return self.position_desired[1]
		
	@property
	def z(self):
		return self.position[2]
		
	@property
	def z_desired(self):
		return self.position_desired[2]
		
	@property
	def rot_matrix(self):
		return self.pose_matrix[0:3,0:3]
		
	@property
	def rotation(self):
		return Rotation.from_matrix(self.rot_matrix)
		
	@property
	def tau(self):
		return self.franka_state_.tau_J
		
	@property
	def tau_desired(self):
		return self.franka_state_.tau_J_d
		
	@property
	def external_torque(self):
		return self.franka_state_.tau_ext_hat_filtered
		
	@property
	def robot(self):
		return self.robot_
		
