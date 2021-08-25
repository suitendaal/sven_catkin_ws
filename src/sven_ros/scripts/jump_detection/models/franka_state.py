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
	def rot_matrix(self):
		return self.pose_matrix[0:3,0:3]
		
	@property
	def rotation(self):
		return Rotation.from_matrix(self.rot_matrix)
		
	@property
	def tau_measured(self):
		return self.franka_state_.tau_J
		
	@property
	def tau_desired(self):
		return self.franka_state_.tau_J_d
		
	@property
	def tau_external(self):
		return self.franka_state_.tau_ext_hat_filtered
		
	@property
	def robot(self):
		return self.robot_
		
	@property
	def jacobian(self):
		return self.robot.jacob0(q=self.q, T=self.robot.fkine(self.q))
		
	@property
	def velocity(self):
		return self.jacobian.dot(np.array(self.dq)).tolist()[0:3]
	
	@property
	def force_measured(self):
		return self.calc_force(self.tau_measured)
		
	@property
	def force_desired(self):
		return self.calc_force(self.tau_desired)
		
	@property
	def force_external(self):
		return self.franka_state_.O_F_ext_hat_K
		
	def calc_force(self, torque):
		return np.linalg.pinv(self.jacobian.T).dot(torque).tolist()[0:3]
	
		
