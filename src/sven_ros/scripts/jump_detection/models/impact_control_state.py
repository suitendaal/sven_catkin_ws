#!/usr/bin/python3

import numpy as np
from scipy.spatial.transform import Rotation
import roboticstoolbox as rtb

class ImpactControlState(object):
	
	def __init__(self, state_msg):
		self.state_ = state_msg
		self.robot_ = rtb.models.DH.Panda()
	
	@property
	def q(self):
		return list(self.state_.q)
		
	@property
	def dq(self):
		return list(self.state_.dq)
		
	@property
	def pose_matrix(self):
		tmp = np.array(self.state_.O_T_EE)
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
		return list(self.state_.position_d)
		
	@property
	def rot_matrix(self):
		return self.pose_matrix[0:3,0:3]
		
	@property
	def rotation(self):
		return Rotation.from_matrix(self.rot_matrix)
		
	@property
	def euler_angles(self):
		return self.rotation.as_euler('xyz').tolist()
		
	@property
	def rotation_desired(self):
		return Rotation.from_quat(self.state_.orientation_d)
		
	@property
	def euler_angles_desired(self):
		return self.rotation_desired.as_euler('xyz').tolist()
		
	@property
	def effort_desired(self):
		return list(self.state_.effort)
		
	@property
	def tau_measured(self):
		return list(self.state_.tau_measured)
		
	@property
	def tau_desired(self):
		return list(self.state_.tau_d)
		
	@property
	def tau_external(self):
		return list(self.state_.tau_external)
		
	@property
	def tau_task(self):
		return list(self.state_.tau_task)
		
	@property
	def tau_nullspace(self):
		return list(self.state_.tau_nullspace)
		
	@property
	def robot(self):
		return self.robot_
		
	@property
	def jacobian(self):
		tmp = np.array(list(self.state_.jacobian))
		shape = (6, 7)
		return tmp.reshape(shape)
		
	@property
	def velocity(self):
		return self.jacobian.dot(np.array(self.dq)).tolist()[0:3]
		
	@property
	def rotational_velocity(self):
		return self.jacobian.dot(np.array(self.dq)).tolist()[3:6]
	
	@property
	def force_measured(self):
		return self.calc_force(self.tau_measured)
		
	@property
	def torque_measured(self):
		return self.calc_torque(self.tau_measured)
		
	@property
	def force_measured_magnitude(self):
		return np.linalg.norm(self.force_measured)
		
	@property
	def force_desired(self):
		return self.calc_force(self.tau_desired)
		
	@property
	def torque_desired(self):
		return self.calc_torque(self.tau_desired)
		
	@property
	def force_external(self):
		return self.calc_force(self.tau_external)
		
	@property
	def torque_external(self):
		return self.calc_torque(self.tau_external)
		
	@property
	def force_external_magnitude(self):
		return np.linalg.norm(self.force_external)
		
	@property
	def force_task(self):
		return self.calc_force(self.tau_task)
		
	@property
	def torque_task(self):
		return self.calc_torque(self.tau_task)
		
	@property
	def control_options(self):
		return self.state_.control_options
		
	def calc_force(self, tau):
		return np.linalg.pinv(self.jacobian.T).dot(np.array(tau)).tolist()[0:3]
		
	def calc_torque(self, tau):
		return np.linalg.pinv(self.jacobian.T).dot(np.array(tau)).tolist()[3:6]
		
	def distance(self, position):
		delta_x = []
		for i in range(3):
			delta_x.append(position[i] - self.position[i])
		return np.linalg.norm(delta_x)
	
		
