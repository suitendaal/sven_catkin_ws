#include <sstream>

#include <vector>

#include "sven_ros/vive_pose_converter_node.h"
#include "jump_detector/datapoint.h"

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
	max_vel_passed(false),
	max_window_length(30),
	filter(3, 0, 20),
	stop_button_pressed(true)
{
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vive_pose_converter/pose", 40);
	vive_sub = nh.subscribe("/vive/pose1", 40, &VivePoseConverterNode::vive_pose_callback, this);
	robot_sub = nh.subscribe("/cartesian_pose", 40, &VivePoseConverterNode::robot_callback, this);
	stop_button_sub = nh.subscribe("/vive/controller_LHR_FF6FDF46/joy", 40, &VivePoseConverterNode::stop_button_callback, this);
	
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
	if (nh.getParam("/vive_pose_converter/scale_factor", scale_factor)) {
		ROS_INFO_STREAM("scale_factor set to " << scale_factor);
	}
}

VivePoseConverterNode::~VivePoseConverterNode() {
	return;
}

void VivePoseConverterNode::run() {
	ros::spin();
}

void VivePoseConverterNode::vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

	if (stop_button_pressed) {
		return;
	}

	ROS_DEBUG_STREAM("Vive message received");
	
	geometry_msgs::PoseStamped vive_pose;
	vive_pose_received(msg, vive_pose);
	
	if (vive_offset_set) {
		geometry_msgs::PoseStamped msg_out;
		if (calculate_pose(msg_out, vive_pose)) {
		
			if (!max_vel_passed && pose_within_velocity_limits(msg_out)) {
				pose_pub.publish(msg_out);
				last_sent_pose = msg_out;
				ROS_DEBUG_STREAM("Message sent");
			}
			else {
				if (!max_vel_passed) {
					ROS_ERROR_STREAM("Message passed max velocity. Restart the node please.");
				}
				max_vel_passed = true;
			}
			max_vel_passed = false;
		}
	}
	else {
		vive_offset.position.x = -vive_pose.pose.position.x;
		vive_offset.position.z = -vive_pose.pose.position.y;
		vive_offset.position.y = -vive_pose.pose.position.z;
		
		vive_offset.orientation.x = -vive_pose.pose.orientation.x;
		vive_offset.orientation.y = -vive_pose.pose.orientation.y;
		vive_offset.orientation.z = -vive_pose.pose.orientation.z;
		vive_offset.orientation.w = -vive_pose.pose.orientation.w;
		
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

void VivePoseConverterNode::stop_button_callback(const sensor_msgs::Joy::ConstPtr& msg) {
	if (msg->buttons[1]) {
		stop_button_pressed = true;
	}
	else {
		stop_button_pressed = false;
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
	max_error = max_vel / 30.0;
	return (abs(msg.pose.position.x - last_sent_pose.pose.position.x) < max_error) && (abs(msg.pose.position.y - last_sent_pose.pose.position.y) < max_error) && (abs(msg.pose.position.z - last_sent_pose.pose.position.z) < max_error);
}

bool VivePoseConverterNode::vive_pose_received(const geometry_msgs::PoseStamped::ConstPtr& msg, geometry_msgs::PoseStamped& vive_pose) {
	vive_poses.push(*msg);
	if (vive_poses.size() > max_window_length + 1) {
		vive_poses.pop();
	}
	
	std::vector<DataPoint> xs;
	std::vector<DataPoint> ys;
	std::vector<DataPoint> zs;
	std::vector<DataPoint> qxs;
	std::vector<DataPoint> qys;
	std::vector<DataPoint> qzs;
	std::vector<DataPoint> qws;
	
	std::queue<geometry_msgs::PoseStamped> q = vive_poses;
	
	while (!q.empty()) {
		geometry_msgs::PoseStamped pose_stamped = q.front();
	
		DataPoint x(pose_stamped.header.stamp.toSec(), pose_stamped.pose.position.x);
		DataPoint y(pose_stamped.header.stamp.toSec(), pose_stamped.pose.position.y);
		DataPoint z(pose_stamped.header.stamp.toSec(), pose_stamped.pose.position.z);
		DataPoint qx(pose_stamped.header.stamp.toSec(), pose_stamped.pose.orientation.x);
		DataPoint qy(pose_stamped.header.stamp.toSec(), pose_stamped.pose.orientation.y);
		DataPoint qz(pose_stamped.header.stamp.toSec(), pose_stamped.pose.orientation.z);
		DataPoint qw(pose_stamped.header.stamp.toSec(), pose_stamped.pose.orientation.w);
		xs.push_back(x);
		ys.push_back(y);
		zs.push_back(z);
		qxs.push_back(qx);
		qys.push_back(qy);
		qzs.push_back(qz);
		qws.push_back(qw);
		
		q.pop();
	}
	
	vive_pose.header = msg->header;
	double x;
	double y;
	double z;
	double qx;
	double qy;
	double qz;
	double qw;
	
	if (filter.filter(xs, x)) {
		vive_pose.pose.position.x = x;
	}
	else {
		vive_pose.pose.position.x = msg->pose.position.x;
	}
	if (filter.filter(ys, y)) {
		vive_pose.pose.position.y = y;
	}
	else {
		vive_pose.pose.position.y = msg->pose.position.y;
	}
	if (filter.filter(zs, z)) {
		vive_pose.pose.position.z = z;
	}
	else {
		vive_pose.pose.position.z = msg->pose.position.z;
	}
	if (filter.filter(qxs, qx)) {
		vive_pose.pose.orientation.x = qx;
	}
	else {
		vive_pose.pose.orientation.x = msg->pose.orientation.x;
	}
	if (filter.filter(qys, qy)) {
		vive_pose.pose.orientation.y = qy;
	}
	else {
		vive_pose.pose.orientation.y = msg->pose.orientation.y;
	}
	if (filter.filter(qzs, qz)) {
		vive_pose.pose.orientation.z = qz;
	}
	else {
		vive_pose.pose.orientation.z = msg->pose.orientation.z;
	}
	if (filter.filter(qws, qw)) {
		vive_pose.pose.orientation.w = qw;
	}
	else {
		vive_pose.pose.orientation.w = msg->pose.orientation.w;
	}
	
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vive_pose_converter");
	VivePoseConverterNode node;
	node.run();
	return 0;
}

