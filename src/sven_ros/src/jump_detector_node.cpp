#include "sven_ros/jump_detector_node.h"
#include "jump_detector/jump_aware_filter.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	return 0;
}
