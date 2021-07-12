#include <sstream>

#include "sven_ros/vive_pose_converter_node.h"

VivePoseConverterNode::VivePoseConverterNode()
: 	nh(),
	x_bounded(false),
	y_bounded(false),
	z_bounded(false),
	x_min(0),
	x_max(0),
	y_min(0),
	y_max(0),
	z_min(0),
	z_max(0),
	vive_offset_set(false),
	robot_offset_set(false),
	scale_factor(0.3),
	max_vel_passed(false)
{
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vive_pose_converter/pose", 40);
	vive_sub = nh.subscribe("/vive/pose1", 40, &VivePoseConverterNode::vive_pose_callback, this);
	robot_sub = nh.subscribe("/cartesian_pose", 40, &VivePoseConverterNode::robot_callback, this);
	
	vive_offset.position.x = 0;
	vive_offset.position.y = 0;
	vive_offset.position.z = 0;
	
	if (nh.getParam("/vive_pose_converter/x_min", x_min)) {
		ROS_INFO_STREAM("X min set to " << x_min);
		x_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/x_max", x_max)) {
		ROS_INFO_STREAM("X max set to " << x_max);
		x_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/y_min", y_min)) {
		ROS_INFO_STREAM("Y min set to " << y_min);
		y_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/y_max", y_max)) {
		ROS_INFO_STREAM("Y max set to " << y_max);
		y_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/z_min", z_min)) {
		ROS_INFO_STREAM("Z min set to " << z_min);
		z_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/z_max", z_max)) {
		ROS_INFO_STREAM("Z max set to " << z_max);
		z_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/max_vel", max_vel)) {
		ROS_INFO_STREAM("Max vel set to " << max_vel);
	}
}

VivePoseConverterNode::~VivePoseConverterNode() {
	return;
}

void VivePoseConverterNode::run() {
	ros::spin();
}

void VivePoseConverterNode::vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_DEBUG_STREAM("Vive message received");
	
	if (vive_offset_set) {
		geometry_msgs::PoseStamped msg_out;
		if (calculate_pose(msg_out, *msg)) {
		
			if (!max_vel_passed && pose_within_velocity_limits(msg_out)) {
				pose_pub.publish(msg_out);
				last_sent_pose = msg_out;
				ROS_DEBUG_STREAM("Message sent");
			}
			else {
				if (!max_vel_passed) {
					ROS_ERROR_STREAM("Message " << msg_out << " passed max velocity. Restart the node please.");
				}
				max_vel_passed = true;
			}
		}
	}
	else {
		vive_offset.position.x = -msg->pose.position.x;
		vive_offset.position.z = -msg->pose.position.y;
		vive_offset.position.y = -msg->pose.position.z;
		
		vive_offset.orientation.x = -msg->pose.orientation.x;
		vive_offset.orientation.y = -msg->pose.orientation.y;
		vive_offset.orientation.z = -msg->pose.orientation.z;
		vive_offset.orientation.w = -msg->pose.orientation.w;
		
		vive_offset_set = true;
	}
}

void VivePoseConverterNode::robot_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_DEBUG_STREAM("Robot message received");
	
	if (!robot_offset_set) {
		robot_offset = msg->pose;
		
		// Set last pose sent to robot pose
		bool tmp = vive_offset_set;
		vive_offset_set = false;
		calculate_pose(last_sent_pose, *msg);
		vive_offset_set = tmp;
		
		robot_offset_set = true;
	}
}

bool VivePoseConverterNode::pose_within_bounds(const geometry_msgs::PoseStamped& msg) {
	if (x_bounded && (msg.pose.position.x < x_min || msg.pose.position.x > x_max)) {
		ROS_DEBUG_STREAM("Outside x bounds");
		return false;
	}
	if (y_bounded && (msg.pose.position.y < y_min || msg.pose.position.y > y_max)) {
		ROS_DEBUG_STREAM("Outside y bounds");
		return false;
	}
	if (z_bounded && (msg.pose.position.z < z_min || msg.pose.position.z > z_max)) {
		ROS_DEBUG_STREAM("Outside z bounds");
		return false;
	}
	return true;
}

bool VivePoseConverterNode::calculate_pose(geometry_msgs::PoseStamped& msg_out, const geometry_msgs::PoseStamped& msg_in) {
	msg_out.header = msg_in.header;
	msg_out.pose = msg_in.pose;
	
	if (vive_offset_set) {
		msg_out.pose.position.y = msg_in.pose.position.z;
		msg_out.pose.position.z = msg_in.pose.position.y;
	
		msg_out.pose.position.x += vive_offset.position.x;
		msg_out.pose.position.y += vive_offset.position.y;
		msg_out.pose.position.z += vive_offset.position.z;
		
		msg_out.pose.position.x *= scale_factor;
		msg_out.pose.position.y *= -scale_factor;
		msg_out.pose.position.z *= scale_factor;
	}
		
	bool result = vive_offset_set && pose_within_bounds(msg_out);
	
	if (robot_offset_set) {
		msg_out.pose.position.x += robot_offset.position.x;
		msg_out.pose.position.y += robot_offset.position.y;
		msg_out.pose.position.z += robot_offset.position.z;
		
		msg_out.pose.orientation.x = robot_offset.orientation.x;
		msg_out.pose.orientation.y = robot_offset.orientation.y;
		msg_out.pose.orientation.z = robot_offset.orientation.z;
		msg_out.pose.orientation.w = robot_offset.orientation.w;
	}
	
	return result && robot_offset_set;
}

bool VivePoseConverterNode::pose_within_velocity_limits(const geometry_msgs::PoseStamped& msg) {
	double max_error = max_vel * (msg.header.stamp.toSec() - last_sent_pose.header.stamp.toSec());
	return (abs(msg.pose.position.x - last_sent_pose.pose.position.x) < max_error) && (abs(msg.pose.position.y - last_sent_pose.pose.position.y) < max_error) && (abs(msg.pose.position.z - last_sent_pose.pose.position.z) < max_error);
	
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vive_pose_converter");
	VivePoseConverterNode node;
	node.run();
	return 0;
}

