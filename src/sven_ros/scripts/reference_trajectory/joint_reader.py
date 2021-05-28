#!/usr/bin/python3

from rosbag_reader import RosbagReader
from dataset import DataSet
from datapoint import DataPoint

class JointReader(RosbagReader):
	"""docstring for JointReader."""

	def __init__(self, bagfile, joint=1):
		super(JointReader, self).__init__(bagfile, "/joint_states")
		self.joint = joint

	def read(self):
		data = DataSet()
		for datapoint in super().read():
			timestamp = datapoint.value.header.stamp.secs * 1000000 + int(datapoint.value.header.stamp.nsecs / 1000)
			pos = datapoint.value.position[self.joint]
			vel = datapoint.value.velocity[self.joint]
			eff = datapoint.value.effort[self.joint]
			data.append(DataPoint(timestamp, [pos, vel, eff]))
		return data

def main():
	reader = JointReader("demo2.bag",1)
	data = reader.read()
	print(data)
	print(data[0][0])

if __name__ == '__main__':
	main()

