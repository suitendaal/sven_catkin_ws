#!/usr/bin/env python3

import rospy
import rospkg
import sys
import json
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Point
from sven_ros.msg import BoolStamped
from sven_ros import ImpedanceControlMode

class SvenRosControllerNode(object):
	def __init__(self, reference_trajectory_file, impact_interval_threshold=0.2, initialized=False):
		if not initialized:
			rospy.init_node('sven_ros_controller', anonymous=True)
		self.reference_trajectory_file = reference_trajectory_file
		self.phases = self.read_reference_trajectory()
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.mode_pub = rospy.Publisher('/impedance_control_mode', Int32, queue_size=40)
		## For debugging
		self.orientation_pub = rospy.Publisher('/orientation', Point, queue_size=40)
		##
		
		# Subscribers
		self.jump_detector_sub = rospy.Subscriber("/sven_ros/jump_detector", BoolStamped, self.jump_detector_callback)
		
		# Time
		self.rate = rospy.Rate(100)
		self.starting_time = 0
		
		# Phase
		self.impact_interval_threshold = impact_interval_threshold
		self.last_jump_time = None
		self.current_phase = 0
		
	@property
	def impact_interval(self):
		if self.current_phase + 1 < len(self.phases):
			return [self.phases[self.current_phase + 1]['time'][0], self.phases[self.current_phase]['time'][-1]]
		return None
		
	@property
	def time_interval(self):
		return [self.phases[0]['time'][0], self.phases[-1]['time'][-1]]
		
	def read_reference_trajectory(self):
		result = []
		with open(self.reference_trajectory_file) as f:
			return json.load(f)['datasets']
		
	def evaluate(self, time):
		phase_data = self.phases[self.current_phase]
		index = 0
		for i in range(len(phase_data['time'])):
			if i == len(phase_data['time']) - 1:
				if time >= phase_data['time'][i]:
					index = i
			else:
				if time >= phase_data['time'][i] and time < phase_data['time'][i+1]:
					index = i
		
		pos_data = []
		vel_data = []
		or_data = []
		for i in range(3):
			pos_data.append(phase_data['position'][index][i])
			vel_data.append(phase_data['velocity'][index][i])
			or_data.append(phase_data['orientation'][index][i])
			
		rot = Rotation.from_euler('xyz', or_data)
		
		## For debugging
		msg = Point()
		msg.x = or_data[0]
		msg.y = or_data[1]
		msg.z = or_data[2]
		self.orientation_pub.publish(msg)
		##
		
		result = []
		result.extend(pos_data)
		result.extend(rot.as_quat().tolist())
		result.extend(vel_data)
		
		return result
		
	def run(self):
		self.starting_time = rospy.get_time()
		while not rospy.is_shutdown():
			time = rospy.get_time() - self.starting_time
			self.update_phase(time)
			
			self.mode_pub.publish(self.get_mode_msg(time))
			
			if time >= self.time_interval[0] and time <= self.time_interval[-1]:
				self.pose_pub.publish(self.get_pose_msg(time))
			
			self.rate.sleep()
			
	def get_pose_msg(self, time):
		msg = PoseStamped()
		x, y, z, qx, qy, qz, qw, xd, yd, zd = self.evaluate(time)
		msg.header.stamp = rospy.Time.from_sec(time + self.starting_time)
		msg.pose.position.x = x
		msg.pose.position.y = y
		msg.pose.position.z = z
		msg.pose.orientation.x = qx
		msg.pose.orientation.y = qy
		msg.pose.orientation.z = qz
		msg.pose.orientation.w = qw
		return msg
		
	def get_mode_msg(self, time):
		msg = Int32()
		msg.data = ImpedanceControlMode.Default
		
		if self.impact_interval is not None:
			if time >= self.impact_interval[0]:
				if self.last_jump_time is not None and self.last_jump_time >= self.impact_interval[0] and time - self.last_jump_time <= self.impact_interval_threshold:
#					msg.data = ImpedanceControlMode.PositionFeedback
					msg.data = ImpedanceControlMode.FeedForward

		return msg
		
	def update_phase(self, time):
		if self.impact_interval is not None:
					
			# Time has past impact interval
			if time > self.impact_interval[-1]:
				rospy.logwarn("Time of impact interval reached.")
				self.current_phase += 1
			elif time >= self.impact_interval[0]:
				if self.last_jump_time is not None and self.last_jump_time >= self.impact_interval[0] and time - self.last_jump_time > self.impact_interval_threshold:
					rospy.loginfo("Last impact of simultaneous impact interval detected.")
					self.current_phase += 1
		
	def jump_detector_callback(self, msg):
		if msg.data:
			rospy.loginfo("Jump detected")
			time = msg.header.stamp.to_sec() - self.starting_time
			if self.last_jump_time is None or time > self.last_jump_time:
				self.last_jump_time = time


if __name__ == '__main__':

	rospy.init_node('sven_ros_controller', anonymous=True)
	trajectory_file = rospy.get_param('~trajectory_file')
	
	rospack = rospkg.RosPack()
	filename = rospack.get_path('sven_ros') + "/" + trajectory_file
	
	print("Filename:",filename)
	
	try:
		node = SvenRosControllerNode(filename)
		node.run()
	except rospy.ROSInterruptException:
		pass
