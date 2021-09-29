#!/usr/bin/env python3

import rospy
import math
import numpy as np
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy

class VivePoseConverterNode(object):
	def __init__(self, vive_scale_factor=1, xlim=(0.3,0.6), ylim=(-0.2,0.4), zlim=(-0.05,0.5), initialized=False):
		if not initialized:
			rospy.init_node('vive_pose_converter', anonymous=True)
			
		# Settings
		self.vive_scale_factor = vive_scale_factor
		self.xlim = xlim
		self.ylim = ylim
		self.zlim = zlim
		
		# Time
		self.rate = rospy.Rate(100)
		
		# Offsets
		self.vive_offset = None
		self.robot_offset = None
		self.button_pressed = True
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.orientation_pub = rospy.Publisher('/equilibrium_orientation', PointStamped, queue_size=40)
		
		# Subscribers
		self.vive_sub = rospy.Subscriber("/vive/pose1", PoseStamped, self.vive_pose_callback)
		self.vive_button_sub = rospy.Subscriber("/vive/controller_LHR_FF6FDF46/joy", Joy, self.vive_button_callback)
		self.robot_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, self.robot_callback)
		
	def run(self):
		rospy.spin()
			
	def vive_pose_callback(self, msg):
	
		if self.button_pressed or self.robot_offset is None:
			return
			
		vive_pose = []
		
		# Position
		vive_pose.extend(convert_vive_position(np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])).tolist())
		
		# Orientation
		or_quat = []
		or_quat.append(msg.pose.orientation.x)
		or_quat.append(msg.pose.orientation.y)
		or_quat.append(msg.pose.orientation.z)
		or_quat.append(msg.pose.orientation.w)
		rot = Rotation.from_quat(or_quat)
#		orientation = convert_vive_orientation(rot.as_matrix())
		orientation = rot.as_matrix()
		vive_pose.append(orientation)
		
		if self.vive_offset is None:
			self.vive_offset = []
			for i in range(3):
				self.vive_offset.append(-vive_pose[i])
			self.vive_offset.append(vive_pose[3].T)
		else:
			pose = []
			
			vive_diff_pose = []
			for i in range(3):
				vive_diff_pose.append((vive_pose[i] + self.vive_offset[i]) * self.vive_scale_factor)
			orientation_diff = Rotation.from_matrix(self.vive_offset[3].dot(vive_pose[3])).as_euler('zxy')
			orientation_diff[0] *= -1
			orientation_diff[1] *= -1
			orientation_diff_matrix = Rotation.from_euler('xyz',orientation_diff).as_matrix()
			vive_diff_pose.append(orientation_diff_matrix)
				
			## DEBUG
			or_eul = Rotation.from_matrix(orientation_diff_matrix).as_euler('xyz')
			msg_out2 = PointStamped()
			msg_out2.header = msg.header
			msg_out2.point.x = or_eul[0]
			msg_out2.point.y = or_eul[1]
			msg_out2.point.z = or_eul[2]
			self.orientation_pub.publish(msg_out2)
			##
				
			for i in range(3):
				pose.append(vive_diff_pose[i] + self.robot_offset[i])
			pose.append(vive_diff_pose[3].dot(self.robot_offset[3]))
			
			if pose[0] < self.xlim[0]:
				pose[0] = self.xlim[0]
			elif pose[0] > self.xlim[-1]:
				pose[0] = self.xlim[-1]
			if pose[1] < self.ylim[0]:
				pose[1] = self.ylim[0]
			elif pose[1] > self.ylim[-1]:
				pose[1] = self.ylim[-1]
			if pose[2] < self.zlim[0]:
				pose[2] = self.zlim[0]
			elif pose[2] > self.zlim[-1]:
				pose[2] = self.zlim[-1]
				
			msg_out = PoseStamped()
			msg_out.header = msg.header
			msg_out.pose.position.x = pose[0]
			msg_out.pose.position.y = pose[1]
			msg_out.pose.position.z = pose[2]
			
			or_mat = Rotation.from_matrix(pose[3])
			or_quat = or_mat.as_quat()
			msg_out.pose.orientation.x = or_quat[0]
			msg_out.pose.orientation.y = or_quat[1]
			msg_out.pose.orientation.z = or_quat[2]
			msg_out.pose.orientation.w = or_quat[3]
			self.pose_pub.publish(msg_out)			
			
		
	def vive_button_callback(self, msg):
		if msg.buttons[1] == 1:
			self.button_pressed = True
			self.vive_offset = None
			self.robot_offset = None
		else:
			self.button_pressed = False
		
	def robot_callback(self, msg):
		if self.button_pressed:
			return
	
		if self.robot_offset is None:
			self.robot_offset = []
			
			# Position
			self.robot_offset.append(msg.pose.position.x)
			self.robot_offset.append(msg.pose.position.y)
			self.robot_offset.append(msg.pose.position.z)
			
			# Orientation
			or_quat = []
			or_quat.append(msg.pose.orientation.x)
			or_quat.append(msg.pose.orientation.y)
			or_quat.append(msg.pose.orientation.z)
			or_quat.append(msg.pose.orientation.w)
			rot = Rotation.from_quat(or_quat)
			self.robot_offset.append(rot.as_matrix())
			
def convert_vive_position(vector):
	rotation_matrix = np.array([
		[0, 0, 1],
		[1, 0, 0],
		[0, 1, 0]
	])
	yaw = math.pi/4
	rotation_matrix2 = np.array([
		[math.cos(yaw), -math.sin(yaw), 0],
		[math.sin(yaw), math.cos(yaw), 0],
		[0, 0, 1]
	])
	return rotation_matrix2.dot(rotation_matrix.dot(vector))
	
def convert_vive_orientation(matrix):
	rotation_matrix = np.array([
		[1, 0, 0],
		[0, 1, 0],
		[0, 0, 1]
	])
	yaw = math.pi/4
	rotation_matrix2 = np.array([
		[math.cos(yaw), -math.sin(yaw), 0],
		[math.sin(yaw), math.cos(yaw), 0],
		[0, 0, 1]
	])
	return rotation_matrix2.dot(rotation_matrix.dot(vector))
	return rotation_matrix.dot(matrix)


if __name__ == '__main__':

	rospy.init_node('vive_pose_converter', anonymous=True)
	
	scale_factor = rospy.get_param('~scale_factor',1)
	x_min = rospy.get_param('~x_min',-1)
	x_max = rospy.get_param('~x_max',1)
	y_min = rospy.get_param('~y_min',-1)
	y_max = rospy.get_param('~y_max',1)
	z_min = rospy.get_param('~z_min',0)
	z_max = rospy.get_param('~z_max',1)
	
	try:
#		node = VivePoseConverterNode()
		node = VivePoseConverterNode(vive_scale_factor=scale_factor, xlim=(x_min,x_max), ylim=(y_min,y_max), zlim=(z_min,z_max), initialized=True)
		node.run()
	except rospy.ROSInterruptException:
		pass
