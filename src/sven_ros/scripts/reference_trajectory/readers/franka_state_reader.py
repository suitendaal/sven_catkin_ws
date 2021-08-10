#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *
from models import FrankaState

class FrankaStateReader(RosbagReader):
	"""docstring for FrankaStateReader."""

	def __init__(self, bagfile, **kwargs):
		super(FrankaStateReader, self).__init__(bagfile, topic="/franka_state_controller/franka_states", **kwargs)

	def read(self):
		data = DataSet(timefactor=1000000)
		for datapoint in super().read():
			timestamp = datapoint.value.header.stamp.secs * 1000000 + int(datapoint.value.header.stamp.nsecs / 1000)
			data.append(DataPoint(timestamp, FrankaState(datapoint.value)), reset_time=True)
		data.align_time()
		return data

