#!/usr/bin/env python3

import rospy
import rospkg
import sys
import json
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Point

class TrajectoryControllerNode(object):
	def __init__(self, reference_trajectory_file, initialized=False):
		if not initialized:
			rospy.init_node('trajectory_controller', anonymous=True)
		self.reference_trajectory_file = reference_trajectory_file
		self.reference_trajectory = self.read_reference_trajectory()
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		
		## For debugging
		self.orientation_pub = rospy.Publisher('/equilibrium_orientation', Point, queue_size=40)
		##
		
		# Time
		self.rate = rospy.Rate(100)
		self.starting_time = 0
		
		# Phase
		self.current_index = 0
		self.trajectory_ended = False
		
	
	@property
	def time_interval(self):
		return [self.reference_trajectory[0]['time'], self.reference_trajectory[-1]['time']]
		
	def read_reference_trajectory(self):
		result = []
		with open(self.reference_trajectory_file) as f:
			return json.load(f)['datapoints']
		
	def evaluate(self, time):
		index_found = False
		for i in range(self.current_index+1, len(self.reference_trajectory)):
			if time < self.reference_trajectory[i]['time']:
				self.current_index = i - 1
				index_found = True
				break
		
		if not index_found:
			self.current_index = -1
			
		value = self.reference_trajectory[self.current_index]['value']
			
		## For debugging
		msg = Point()
		msg.x = value[3]
		msg.y = value[4]
		msg.z = value[5]
		self.orientation_pub.publish(msg)
		##
		
		result = value[0:3].copy()
		rot = Rotation.from_euler('xyz', value[3:6])
		result.extend(rot.as_quat().tolist())
		return result
		
	def run(self):
		self.starting_time = rospy.get_time()
		while not rospy.is_shutdown():
			time = rospy.get_time() - self.starting_time
			
			if time >= self.reference_trajectory[0]['time'] and not self.trajectory_ended:
				self.pose_pub.publish(self.get_pose_msg(time))
				
			if time >= self.reference_trajectory[-1]['time'] and not self.trajectory_ended:
				self.trajectory_ended = True
				rospy.loginfo("Trajectory ended at time {}.".format(time))
			
			self.rate.sleep()
			
	def get_pose_msg(self, time):
		msg = PoseStamped()
		x, y, z, qx, qy, qz, qw = self.evaluate(time)
		msg.header.stamp = rospy.Time.from_sec(time + self.starting_time)
		msg.pose.position.x = x
		msg.pose.position.y = y
		msg.pose.position.z = z
		msg.pose.orientation.x = qx
		msg.pose.orientation.y = qy
		msg.pose.orientation.z = qz
		msg.pose.orientation.w = qw
		return msg
		

if __name__ == '__main__':

	rospy.init_node('trajectory_controller', anonymous=True)
	trajectory_file = rospy.get_param('~trajectory_file')
	
	rospack = rospkg.RosPack()
	filename = rospack.get_path('sven_ros') + "/" + trajectory_file
	
	print("Filename:",filename)
	
	try:
		node = TrajectoryControllerNode(filename)
		node.run()
	except rospy.ROSInterruptException:
		pass
