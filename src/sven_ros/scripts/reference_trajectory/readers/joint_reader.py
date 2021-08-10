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
			timestamp = datapoint.value.header.stamp.secs * 1000000 + int(datapoint.value.header.stamp.nsecs / 1000)
			pos = datapoint.value.q[self.joint - 1]
			vel = datapoint.value.dq[self.joint - 1]
			eff = datapoint.value.tau_J[self.joint - 1]
			data.append(DataPoint(timestamp, [pos, vel, eff]), reset_time=True)
		return data

