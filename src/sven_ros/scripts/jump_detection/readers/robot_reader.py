#!/usr/bin/python3

from .rosbag_reader import RosbagReader
from datalib import *
from models import RobotState

class RobotReader(RosbagReader):
	"""docstring for FrankaStateReader."""

	def __init__(self, bagfile, **kwargs):
		super(RobotReader, self).__init__(bagfile, topic="/impact_aware_cartesian_impedance_controller/impact_control_state", **kwargs)

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			data.append(DataPoint(datapoint.time, RobotState(datapoint.value)))
		return data

