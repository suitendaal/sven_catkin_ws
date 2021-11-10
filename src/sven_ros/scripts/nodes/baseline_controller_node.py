#!/usr/bin/env python3

import rospy
import rospkg
import sys
import json
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from sven_ros.msg import BoolStamped
from franka_custom_controllers.msg import ControlOptions, RobotState

class BaselineControllerNode(object):
	def __init__(self, reference_trajectory_file, initialized=False):
		if not initialized:
			rospy.init_node('baseline_controller', anonymous=True)
		self.reference_trajectory_file = reference_trajectory_file
		self.phases = self.read_reference_trajectory()
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_state', RobotState, queue_size=40)
		self.mode_pub = rospy.Publisher('/impedance_control_options', ControlOptions, queue_size=40)
		## For debugging
		self.orientation_pub = rospy.Publisher('/orientation', Point, queue_size=40)
		##
		
		# Time
		self.rate = rospy.Rate(100)
		self.starting_time = 0
		
		# Phase
		self.current_phase = 0
		self.trajectory_ended = False
		
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
		for_data = []
		tor_data = []
		for i in range(3):
			pos_data.append(phase_data['position'][index][i])
			vel_data.append(phase_data['velocity'][index][i])
			or_data.append(phase_data['orientation'][index][i])
			for_data.append(phase_data['force'][index][i])
			tor_data.append(phase_data['torque'][index][i])
			
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
		result.extend(for_data)
		result.extend(tor_data)
		
		return result
		
	def run(self):
		self.starting_time = rospy.get_time()
		while not rospy.is_shutdown():
			time = rospy.get_time() - self.starting_time
			self.update_phase(time)
			
			if not self.trajectory_ended:
				self.mode_pub.publish(self.get_mode_msg(time))
			
			if time >= self.time_interval[0] and not self.trajectory_ended:
				self.pose_pub.publish(self.get_pose_msg(time))
				
			if time > self.time_interval[-1] and not self.trajectory_ended:
				self.trajectory_ended = True
				rospy.loginfo("Trajectory ended at time {}.".format(time))
				self.rate.sleep()
				break
			
			self.rate.sleep()
			
	def get_pose_msg(self, time):
		msg = RobotState()
		x, y, z, qx, qy, qz, qw, xd, yd, zd, fx, fy, fz, tx, ty, tz = self.evaluate(time)
		msg.header.stamp = rospy.Time.from_sec(time + self.starting_time)
		msg.pose.position.x = x
		msg.pose.position.y = y
		msg.pose.position.z = z
		msg.pose.orientation.x = qx
		msg.pose.orientation.y = qy
		msg.pose.orientation.z = qz
		msg.pose.orientation.w = qw
		msg.velocity.linear.x = xd
		msg.velocity.linear.y = yd
		msg.velocity.linear.z = zd
		msg.effort.linear.x = fx
		msg.effort.linear.y = fy
		msg.effort.linear.z = fz
		msg.effort.angular.x = tx
		msg.effort.angular.y = ty
		msg.effort.angular.z = tz
		
		return msg
		
	def get_mode_msg(self, time):
		msg = ControlOptions()
		msg.header.stamp = rospy.Time.from_sec(time + self.starting_time)			
		msg.use_position_feedback = True
		msg.use_velocity_feedback = True
		msg.use_velocity_feedforward = False
		msg.use_acceleration_feedforward = False
		msg.stiffness_type = 0
		msg.use_torque_saturation = True
		msg.delta_tau_max = 1.0
		msg.use_effort_feedforward = False
#		# TODO: dependent on phase
#		msg.use_effort_feedforward = False
#		if self.current_phase == 1:
#			msg.use_effort_feedforward = True
		
		return msg
		
	def update_phase(self, time):
		if self.impact_interval is not None:
			
			# In the middle of 'impact interval'
			if time > (self.impact_interval[0] + self.impact_interval[-1]) / 2:
				rospy.loginfo("Switching to next phase at time {}.".format(time))
				self.current_phase += 1


if __name__ == '__main__':

	rospy.init_node('baseline_controller', anonymous=True)
	trajectory_file = rospy.get_param('~trajectory_file')
	
	rospack = rospkg.RosPack()
	filename = rospack.get_path('sven_ros') + "/" + trajectory_file
	
	print("Filename:",filename)
	
	try:
		node = BaselineControllerNode(filename)
		node.run()
	except rospy.ROSInterruptException:
		pass
