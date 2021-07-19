#!/usr/bin/env python3

import rospy
import sys
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from sven_ros.msg import BoolStamped
from sven_ros import ImpedanceControlMode
from sven_ros import EndEffectorHandler

class SvenRosControllerNode(object):
	def __init__(self, end_effector, impact_interval_threshold=0.1):
		rospy.init_node('sven_ros_controller', anonymous=True)
		self.end_effector = end_effector
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.mode_pub = rospy.Publisher('/impedance_control_mode', Int32, queue_size=40)
		
		# Subscribers
		self.jump_detector_subs = []
		for i in range(7):
			self.jump_detector_subs.append(rospy.Subscriber("/sven_ros/jump_detector" + str(i+1), BoolStamped, self.jump_detector_callback))
			
		
		# Time
		self.rate = rospy.Rate(30)
		self.starting_time = 0
		
		# Phase
		self.impact_interval_threshold = impact_interval_threshold
		self.last_jump_time = None
		
	def run(self):
		time_interval = self.end_effector.get_time_interval()
	
		self.starting_time = rospy.get_time()
		while not rospy.is_shutdown():
			time = rospy.get_time() - self.starting_time
			self.update_phase(time)
			
			self.mode_pub.publish(self.get_mode_msg(time))
			
			if time >= time_interval[0] and time <= time_interval[1]:
				self.pose_pub.publish(self.get_pose_msg(time))
			
			self.rate.sleep()
			
	def get_pose_msg(self, time):
		msg = PoseStamped()
		[x, y, z, qx, qy, qz, qw, xd, yd, zd] = self.end_effector.evaluate(time)
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
		
		if self.end_effector.phase + 1 != self.end_effector.n_phases:
			impact_interval = self.end_effector.get_impact_intervals()[self.end_effector.phase]
			if time >= impact_interval[0]:
				if self.last_jump_time is not None and self.last_jump_time >= impact_interval[0] and time - self.last_jump_time <= self.impact_interval_threshold:
					msg.data = ImpedanceControlMode.PositionFeedback
		
		return msg
		
	def update_phase(self, time):
		if self.end_effector.phase + 1 != self.end_effector.n_phases:
			impact_interval = self.end_effector.get_impact_intervals()[self.end_effector.phase]
			
			# Time has past impact interval
			if time > impact_interval[1]:
				rospy.logwarn("Time of impact interval reached.")
				self.end_effector.update_phase()
			elif time >= impact_interval[0]:
				if self.last_jump_time is not None and self.last_jump_time >= impact_interval[0] and time - self.last_jump_time > self.impact_interval_threshold:
					rospy.loginfo("Last impact of simultaneous impact interval detected.")
					self.end_effector.update_phase()
		
		
	def jump_detector_callback(self, msg):
		if msg.data:
			time = msg.header.stamp.to_sec()
			if time > self.last_jump_time:
				self.last_jump_time = time


if __name__ == '__main__':
	if len(sys.argv) > 3:
		filename = sys.argv[1]
		ee = EndEffectorHandler(filename)
		
		try:
			node = SvenRosControllerNode(ee)
			node.run()
		except rospy.ROSInterruptException:
			pass
