#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *

class JointReader(RosbagReader):
	"""docstring for JointReader."""

	def __init__(self, bagfile, joint=1, **kwargs):
		self.joint = joint
		super(JointReader, self).__init__(bagfile, topic="/franka_state_controller/franka_states", **kwargs)

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			pos = datapoint.value.q[self.joint - 1]
			vel = datapoint.value.dq[self.joint - 1]
			eff = datapoint.value.tau_J[self.joint - 1]
			data.append(DataPoint(datapoint.time, [pos, vel, eff]))
		return data

