#!/usr/bin/env python3

import rospy
import rospkg
import dynamic_reconfigure.client
import sys
import json
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Point

class TrajectoryControllerNode(object):
	def __init__(self, reference_trajectory_file, impact_bounds, rotational_stiffnesses, rate=100, reconfigure_node='dynamic_reconfigure_compliance_param_node', initialized=False):
		if not initialized:
			rospy.init_node('trajectory_controller', anonymous=True)
		self.reference_trajectory_file = reference_trajectory_file
		self.reference_trajectory = self.read_reference_trajectory()
		self.impact_bounds = impact_bounds
		self.rotational_stiffnesses = rotational_stiffnesses
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.client = dynamic_reconfigure.client.Client(reconfigure_node)
		
		## For debugging
		self.orientation_pub = rospy.Publisher('/equilibrium_orientation', Point, queue_size=40)
		##
		
		# Time
		self.rate = rospy.Rate(rate)
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
		
		self.check_bounds(result)
		
		rot = Rotation.from_euler('xyz', value[3:6])
		result.extend(rot.as_quat().tolist())
		return result
		
	def check_bounds(self, position):
		in_impact = False
		for i in range(3):
			index = 2*i
			if position[i] >= self.impact_bounds[index] and position[i] <= self.impact_bounds[index+1]:
				in_impact = True
				break
				
		if in_impact:
			rotational_stiffness = self.rotational_stiffnesses[1]
		else:
			rotational_stiffness = self.rotational_stiffnesses[0]
		params = { 'rotational_stiffness' : rotational_stiffness }
		config = self.client.update_configuration(params)
		
	def run(self):
		self.starting_time = rospy.get_time()
		while not rospy.is_shutdown() and not self.trajectory_ended:
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
	impact_bounds = rospy.get_param('~impact_bounds')
	rotational_stiffness = rospy.get_param('~rotational_stiffness')
	rotational_stiffness_impact = rospy.get_param('~rotational_stiffness_impact')
	
	rospack = rospkg.RosPack()
	filename = rospack.get_path('sven_ros') + "/" + trajectory_file
	
	print("Filename:",filename)
	
	try:
		node = TrajectoryControllerNode(filename, impact_bounds, [rotational_stiffness, rotational_stiffness_impact])
		rospy.sleep(1)
		node.run()
	except rospy.ROSInterruptException:
		pass
