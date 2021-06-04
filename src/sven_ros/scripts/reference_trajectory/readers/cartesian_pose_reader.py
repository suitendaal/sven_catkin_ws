#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *

class CartesianPoseReader(RosbagReader):
	"""docstring for CartesianPoseReader."""

	def __init__(self, bagfile):
		super(CartesianPoseReader, self).__init__(bagfile, "/cartesian_pose")

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			timestamp = datapoint.value.header.stamp.secs * 1000000 + int(datapoint.value.header.stamp.nsecs / 1000)
			x = datapoint.value.pose.position.x
			y = datapoint.value.pose.position.y
			z = datapoint.value.pose.position.z
			a = datapoint.value.pose.orientation.x
			b = datapoint.value.pose.orientation.y
			c = datapoint.value.pose.orientation.z
			d = datapoint.value.pose.orientation.w
			data.append(DataPoint(timestamp, [x, y, z, a, b, c, d]))
		return data

