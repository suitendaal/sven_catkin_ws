#!/usr/bin/python3

from readers.rosbag_reader import RosbagReader
from datalib.dataset import DataSet
from datalib.datapoint import DataPoint

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

def main():
	reader = CartesianPoseReader("traj6.1_5.bag")
	data = reader.read()
	print(data[0][0])

if __name__ == '__main__':
	main()

