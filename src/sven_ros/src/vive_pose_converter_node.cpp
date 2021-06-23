#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

class VivePoseConverterNode {
public:
	ros::NodeHandle nh;
	
	VivePoseConverterNode();
    ~VivePoseConverterNode();
	void run();

private:
	ros::Publisher pose_pub;
	ros::Subscriber vive_sub;
	
	double x_min;
	double x_max;
	double y_min;
	double y_max;
	double z_min;
	double z_max;
	bool x_bounded;
	bool y_bounded;
	bool z_bounded;
	
	void vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool pose_within_bounds(const geometry_msgs::PoseStamped& msg);
};

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
	z_max(0)
{
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vive_pose_converter/pose", 1000);
	vive_sub = nh.subscribe("/vive/pose1", 1000, &VivePoseConverterNode::vive_pose_callback, this);
	
	if (nh.getParam("/vive_pose_converter/x_min", x_min)) {
		ROS_INFO_STREAM("X min received: " << x_min);
		x_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/x_max", x_max)) {
		ROS_INFO_STREAM("X max received: " << x_max);
		x_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/y_min", y_min)) {
		ROS_INFO_STREAM("Y min received: " << y_min);
		y_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/y_max", y_max)) {
		ROS_INFO_STREAM("Y max received: " << y_max);
		y_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/z_min", z_min)) {
		ROS_INFO_STREAM("Z min received: " << z_min);
		z_bounded = true;
	}
	if (nh.getParam("/vive_pose_converter/z_max", z_max)) {
		ROS_INFO_STREAM("Z max received: " << z_max);
		z_bounded = true;
	}
}

VivePoseConverterNode::~VivePoseConverterNode() {
	return;
}

void VivePoseConverterNode::run() {
	ros::spin();
}

void VivePoseConverterNode::vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	ROS_DEBUG_STREAM("Message received");
	if (pose_within_bounds(*msg))
	{
		pose_pub.publish(*msg);
		ROS_DEBUG_STREAM("Message sent");
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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vive_pose_converter");
	VivePoseConverterNode node;
	node.run();
	return 0;
}

