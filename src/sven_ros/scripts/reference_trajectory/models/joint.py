#!/usr/bin/python3

from .joint_data import *

class Joint(object):
	"""docstring for Joint."""

	def __init__(self, number, jump_detector=None, **kwargs):
		self.number = number
		self.data = []
		self.joint_data = []
		self.jump_detector = jump_detector
		
	def append_data(self, data):
		self.data.append(data)
		self.joint_data.append(JointData(data))
		
	def detect_jumps(self, datasets=None):
		if datasets is None:
			datasets = range(len(joint_data))
		jumps = []
		for i in datasets:
			jumps.append(self.joint_data[i].detect_jumps(self.jump_detector))
		return jumps
			
		
