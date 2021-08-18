#!/usr/bin/python3

import numpy as np
from scipy.spatial.transform import Rotation

class RobotState(object):
	def __init__(self, robot_state_msg):
		self.msg_ = robot_state_msg
		
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
	def position(self):
		mat = self.pose_matrix
		result = []
		for i in range(3):
			result.append(mat[i,3])
		return result
		
	@property
	def position_desired(self):
		return self.msg_.position_d
		
	@property
	def velocity(self):
		return self.jacobian.dot(np.array(self.dq)).tolist()[0:3]
		
	@property
	def rotation_matrix(self):
		return self.pose_matrix[0:3,0:3]
		
	@property
	def rotation(self):
		return Rotation.from_matrix(self.rot_matrix)
		
	@property
	def orientation_desired(self):
		return self.msg_.orientation_d
		
	@property
	def tau_external(self):
		return self.msg_.tau_external
		
	@property
	def tau_measured(self):
		return self.msg_.tau_measured
		
	@property
	def tau_task(self):
		return self.msg_.tau_task
		
	@property
	def tau_nullspace(self):
		return self.msg_.tau_nullspace
		
	@property
	def tau_coriolis(self):
		return self.msg_.coriolis
		
	# Saturated desired torque
	@property
	def tau_desired_saturated(self):
		return self.msg_.tau_d

	@property
	def tau_desired(self):
		result = []
		for i in range(len(self.tau_task)):
			value = self.tau_task[i]
			value += self.tau_nullspace[i]
			value += self.tau_coriolis[i]
			result.append(value)
		return result
		
	@property
	def force_external(self):
		return self.calc_force(self.tau_external)
		
	@property
	def force_measured(self):
		return self.calc_force(self.tau_measured)
		
	@property
	def force_task(self):
		return self.calc_force(self.tau_task)
		
	@property
	def force_nullspace(self):
		return self.calc_force(self.tau_nullspace)
		
	@property
	def force_coriolis(self):
		return self.calc_force(self.tau_coriolis)
		
	@property
	def force_desired_saturated(self):
		return self.calc_force(self.tau_desired_saturated)
		
	@property
	def force_desired(self):
		return self.calc_force(self.tau_desired)
		
	def calc_force(self, torque):
		return np.linalg.pinv(self.jacobian.T).dot(torque).tolist()[0:3]
		
	@property
	def impedance_control_mode(self):
		return self.msg_.impedance_control_mode
		
