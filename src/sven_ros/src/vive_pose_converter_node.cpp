#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"

#include <sstream>


void vive_pose_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	ROS_INFO("Message received");
	// TODO: convert twist to pose with bounds
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "vive_pose_converter");
	ros::NodeHandle nh;

	ros::Publisher chatter_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_input_pose", 1000);
	ros::Subscriber sub = nh.subscribe("/vive/twist1", 1000, vive_pose_callback);

	ros::spin();

	return 0;
}

