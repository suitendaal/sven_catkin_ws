#!/usr/bin/env python3

import rospy
import math
from scipy.spatial.transform import Rotation
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Joy

class VivePoseConverterNode(object):
	def __init__(self, vive_scale_factor=1, xlim=(-0.5,0.5), ylim=(-0.5,0.5), zlim=(-0.5,0.5), initialized=False):
		if not initialized:
			rospy.init_node('vive_pose_converter', anonymous=True)
			
		# Settings
		self.vive_scale_factor = vive_scale_factor
		self.xlim = xlim
		self.ylim = ylim
		self.zlim = zlim
		
		# Publishers
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.orientation_pub = rospy.Publisher('/equilibrium_orientation', PointStamped, queue_size=40)
		
		# Subscribers
		self.vive_sub = rospy.Subscriber("/vive/pose1", PoseStamped, self.vive_pose_callback)
		self.vive_button_sub = rospy.Subscriber("/vive/controller_LHR_FF6FDF46/joy", Joy, self.vive_button_callback)
		self.robot_sub = rospy.Subscriber("/cartesian_pose", PoseStamped, self.robot_callback)
		
		# Time
		self.rate = rospy.Rate(100)
		
		# Offsets
		self.vive_offset = None
		self.robot_offset = None
		self.button_pressed = True
		
	def run(self):
		rospy.spin()
			
	def vive_pose_callback(self, msg):
		if self.robot_offset is None or self.button_pressed:
			return
			
		vive_pose = []
		
		# Position
		vive_pose.append(msg.pose.position.z * math.cos(math.pi / 4) - msg.pose.position.x * math.sin(math.pi / 4))
		vive_pose.append(msg.pose.position.x * math.cos(math.pi / 4) + msg.pose.position.z * math.sin(math.pi / 4))
		vive_pose.append(msg.pose.position.y)
		
		# Orientation
		or_quat = []
		or_quat.append(msg.pose.orientation.x)
		or_quat.append(msg.pose.orientation.y)
		or_quat.append(msg.pose.orientation.z)
		or_quat.append(msg.pose.orientation.w)
		rot = Rotation.from_quat(or_quat)
		orientation = rot.as_euler('xyz')
		vive_pose.append(orientation[2])
		vive_pose.append(-orientation[0])
		vive_pose.append(-orientation[1] + math.pi / 4)
		
		if self.vive_offset is None:
			self.vive_offset = []
			for i in vive_pose:
				self.vive_offset.append(-i)
		else:
			pose = []
			for i in range(3):
				pose.append((vive_pose[i] + self.vive_offset[i]) * self.vive_scale_factor + self.robot_offset[i])
			for i in range(3,6):
				pose.append(vive_pose[i] + self.vive_offset[i] + self.robot_offset[i])
			
			if pose[0] < xlim[0] or pose[0] > xlim[-1]
				return
			if pose[1] < ylim[0] or pose[1] > ylim[-1]
				return
			if pose[2] < zlim[0] or pose[2] > zlim[-1]
				
			msg_out = PoseStamped()
			msg_out.header = msg.header
			msg_out.pose.position.x = pose[0]
			msg_out.pose.position.y = pose[1]
			msg_out.pose.position.z = pose[2]
			
			or_eul = Rotation.from_euler('xyz',pose[3:6])
			or_quat = or_eul.as_quat()
			msg_out.pose.orientation.x = or_quat[0]
			msg_out.pose.orientation.y = or_quat[1]
			msg_out.pose.orientation.z = or_quat[2]
			msg_out.pose.orientation.w = or_quat[3]
			self.pose_pub.publish(msg_out)
			
			msg_out2 = PointStamped()
			msg_out2.header = msg_out.header
			msg_out2.point.x = pose[3]
			msg_out2.point.y = pose[4]
			msg_out2.point.z = pose[5]
			self.orientation_pub.publish(msg_out2)
			
			
		
	def vive_button_callback(self, msg):
		if msg.buttons[1] == 1:
			self.button_pressed = True
			self.vive_offset = None
		else:
			self.button_pressed = False
		
	def robot_callback(self, msg):
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
			self.robot_offset.extend(rot.as_euler('xyz'))


if __name__ == '__main__':

	rospy.init_node('vive_pose_converter', anonymous=True)
	
	scale_factor = rospy.get_param('~scale_factor')
	x_min = rospy.get_param('~x_min')
	x_max = rospy.get_param('~x_max')
	y_min = rospy.get_param('~y_min')
	y_max = rospy.get_param('~y_max')
	z_min = rospy.get_param('~z_min')
	z_max = rospy.get_param('~z_max')
	
	try:
		node = VivePoseConverterNode(vive_scale_factor=scale_factor, xlim=(x_min,x_max), ylim=(y_min,y_max), zlim=(z_min,z_max), initialized=True)
		node.run()
	except rospy.ROSInterruptException:
		pass
