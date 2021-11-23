#!/usr/bin/env python3

import rospy
from franka_msgs.msg import FrankaState
from sven_ros.msg import BoolStamped
from sven_ros.jump_detection import *

class ForceRateImpactDetectorNode(object):
	def __init__(self, jump_detector, initialized=False):
		if not initialized:
			rospy.init_node('force_rate_impact_detector_node', anonymous=True)
		
		self.jump_detector = jump_detector
		
		# Publishers
		self.jump_detection_pub = rospy.Publisher('jump_detector', BoolStamped, queue_size=40)
		
		# Subscribers
		self.data_sub = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.data_received)
		
		# Settings
		self.starting_time = None
		
	def data_received(self, msg):
		if self.starting_time is None:
			self.starting_time = msg.header.stamp.to_sec()
		
		time = msg.header.stamp.to_sec() - self.starting_time
		value = list(msg.O_F_ext_hat_K)[0:3]
		jump_detected, info = self.jump_detector.update(PositionDataPoint(time, value))
		
		msg_out = BoolStamped()
		msg_out.header.stamp = msg.header.stamp
		msg_out.data = jump_detected
		self.jump_detection_pub.publish(msg_out)
		
		if jump_detected:
			print(f'Jump detected at time {msg.header.stamp.to_sec()}')
			
	
	def run(self):
		rospy.spin()
		
if __name__ == '__main__':

	rospy.init_node('force_rate_impact_detector_node', anonymous=True)
	
	time_window = rospy.get_param('~predictor_config/time_window')
	predictor = MultiForceDerivativePredictor(time_window=time_window)
	
	bound = rospy.get_param('~bounder_config/bound')
	bounder = ForceDerivativeBounder(bound=bound)
	
	max_window_length = rospy.get_param('~ja_filter_config/max_window_length')
	ja_filter = MultiExternalForceJumpAwareFilter(predictor, bounder, max_window_length=max_window_length)

	node = ForceRateImpactDetectorNode(ja_filter)
	node.run()
	
