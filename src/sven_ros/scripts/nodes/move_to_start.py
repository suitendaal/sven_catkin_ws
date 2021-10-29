#!/usr/bin/env python3

import rospy
import rospkg
import sys
import json
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseStamped, Point


if __name__ == '__main__':

	rospy.init_node('move_to_start', anonymous=True)
	trajectory_file = rospy.get_param('~trajectory_file')
	
	rospack = rospkg.RosPack()
	filename = rospack.get_path('sven_ros') + "/" + trajectory_file
	
	print("Filename:",filename)
	
	try:
		pose_pub = rospy.Publisher('/equilibrium_pose', PoseStamped, queue_size=40)
		orientation_pub = rospy.Publisher('/equilibrium_orientation', Point, queue_size=40)
		rospy.sleep(1)
		
		with open(filename) as f:
			value = json.load(f)['datapoints'][0]['value']
			
			## For debugging
			msg1 = Point()
			msg1.x = value[3]
			msg1.y = value[4]
			msg1.z = value[5]
			orientation_pub.publish(msg1)
			##
			
			msg = PoseStamped()
			msg.header.stamp = rospy.get_rostime()
			msg.header.frame_id = "move_to_start"
			msg.pose.position.x = value[0]
			msg.pose.position.y = value[1]
			msg.pose.position.z = value[2]
			
			rot = Rotation.from_euler('xyz', value[3:6]).as_quat().tolist()
			
			msg.pose.orientation.x = rot[0]
			msg.pose.orientation.y = rot[1]
			msg.pose.orientation.z = rot[2]
			msg.pose.orientation.w = rot[3]
			
			pose_pub.publish(msg)
			rospy.sleep(1)
			print("Message sent")
		print("File closed")
		
	except rospy.ROSInterruptException:
		print("Failed")
	
	
	sys.exit(0)
