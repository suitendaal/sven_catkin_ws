#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *
from models import FrankaState

class FrankaStateReader(RosbagReader):
	"""docstring for FrankaStateReader."""

	def __init__(self, bagfile, **kwargs):
		super(FrankaStateReader, self).__init__(bagfile, topic="/franka_state_controller/franka_states", **kwargs)

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			data.append(DataPoint(datapoint.time, FrankaState(datapoint.value)))
		return data

