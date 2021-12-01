#include <sven_ros/baseline_controller_node.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "baseline_controller_node");
	ros::NodeHandle nh("~");
	
	std::string reference_trajectory_file;
	nh.param<std::string>("trajectory_file", reference_trajectory_file, "");
	
	double publish_rate;
	nh.param<double>("config/publish_rate", publish_rate, 30.0);
	
  BaselineControllerNode controller(nh, reference_trajectory_file, publish_rate);
  return controller.run();
}
