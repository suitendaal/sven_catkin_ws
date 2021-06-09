#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *

class JointReader(RosbagReader):
	"""docstring for JointReader."""

	def __init__(self, bagfile, joint=1, **kwargs):
		super(JointReader, self).__init__(bagfile, topic="/joint_states", **kwargs)
		self.joint = joint

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			timestamp = datapoint.value.header.stamp.secs * 1000000 + int(datapoint.value.header.stamp.nsecs / 1000)
			pos = datapoint.value.position[self.joint - 1]
			vel = datapoint.value.velocity[self.joint - 1]
			eff = datapoint.value.effort[self.joint - 1]
			data.append(DataPoint(timestamp, [pos, vel, eff]))
		return data

