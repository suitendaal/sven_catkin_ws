#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

class SvenRosControllerNode(object):
	def __init__(self):
		rospy.init_node('sven_ros_controller', anonymous=True)
		self.pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		self.mode_pub = rospy.Publisher('/impedance_control_mode', Int32, queue_size=40)
		self.rate = rospy.Rate(30)
		
	def run(self):
		while not rospy.is_shutdown():
			self.pose_pub.publish(self.get_pose_msg())
			self.rate.sleep()
			
	def get_pose_msg(self):
		msg = PoseStamped()
		return msg
		


if __name__ == '__main__':
	try:
		node = SvenRosControllerNode()
		node.run()
	except rospy.ROSInterruptException:
		pass
