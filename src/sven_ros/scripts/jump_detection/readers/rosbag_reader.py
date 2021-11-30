#!/usr/bin/python3

import rosbag
from datalib import *

class RosbagReader(object):
	"""docstring for RosbagReader."""

	def __init__(self, bagfile, topic=None, **kwargs):
		super(RosbagReader, self).__init__()
		self.bagfile = bagfile
		self.topic = topic
		self.use_record_time = kwargs.get('use_record_time', False)
		self.msgs = self.read()
		self.msgs_index_ = 0

	def read(self, topic=None):
		if topic is None:
			topic = self.topic
		msgs = DataSet()
		bag = rosbag.Bag(self.bagfile)
		for top, msg, t in bag.read_messages(topics=[topic]):
			if self.use_record_time:
				time = t.secs + int(t.nsecs) / 1000000000
			else:
				time = msg.header.stamp.secs + int(msg.header.stamp.nsecs) / 1000000000
			msgs.append(DataPoint(time, msg))
		bag.close()
		return msgs
		
	def next_datapoint(self):
		if self.msgs_index_ < len(self.msgs):
			result = self.msgs[self.msgs_index_]
			self.msgs_index_ += 1
			return result
		return None
		
	@property
	def first_datapoint(self):
		if len(self.msgs) > 0:
			return self.msgs[0]
		return None
		
	@property
	def last_datapoint(self):
		if len(self.msgs) > 0:
			return self.msgs[-1]
		return None
		
	def reset(self):
		self.msgs_index_ = 0
		
	def align_time(self):
		return self.msgs.align_time
		
	def end(self):
		return self.msgs_index_ >= len(self.msgs)

def main():
	reader = RosbagReader("demo2.bag")
	msgs = reader.read("/joint_states")
	print(msgs)

if __name__ == '__main__':
	main()
