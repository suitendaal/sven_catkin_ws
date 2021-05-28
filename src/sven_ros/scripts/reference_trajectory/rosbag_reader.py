#!/usr/bin/python3

import rosbag
from datapoint import DataPoint

class RosbagReader(object):
	"""docstring for RosbagReader."""

	def __init__(self, bagfile, topic=None):
		super(RosbagReader, self).__init__()
		self.bagfile = bagfile
		self.topic = topic

	def read(self, topic=None):
		if topic is None:
			topic = self.topic
		msgs = []
		bag = rosbag.Bag(self.bagfile)
		for top, msg, t in bag.read_messages(topics=[topic]):
			msgs.append(DataPoint(t.secs * 1000000 + int(t.nsecs / 1000), msg))
		bag.close()
		return msgs

def main():
	reader = RosbagReader("demo2.bag")
	msgs = reader.read("/joint_states")
	print(msgs)

if __name__ == '__main__':
	main()
