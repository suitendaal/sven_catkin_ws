#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>

class VivePoseConverterNode
{
public:
	ros::NodeHandle nh;
	
	VivePoseConverterNode();
    ~VivePoseConverterNode();
	void run();

private:
	ros::Publisher pose_pub;
	ros::Subscriber vive_sub;
	
	void vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	bool pose_within_bounds(const geometry_msgs::PoseStamped::ConstPtr& msg);
};

VivePoseConverterNode::VivePoseConverterNode()
: nh()
{
	pose_pub = nh.advertise<geometry_msgs::PoseStamped>("robot_input_pose", 1000);
	vive_sub = nh.subscribe("/vive/pose1", 1000, &VivePoseConverterNode::vive_pose_callback, this);
}

VivePoseConverterNode::~VivePoseConverterNode()
{
	return;
}

void VivePoseConverterNode::run()
{
	ros::spin();
}

void VivePoseConverterNode::vive_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ROS_INFO("Vive pose received");
	if (pose_within_bounds(msg))
	{
		pose_pub.publish(msg);
	}
}

bool VivePoseConverterNode::pose_within_bounds(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vive_pose_converter");
	VivePoseConverterNode node;
	node.run();
	return 0;
}

