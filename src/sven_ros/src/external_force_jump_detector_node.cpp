#include <sven_ros/external_force_jump_detector_node.h>
#include <jump_detector/jump_aware_filter.h>
#include <jump_detector/constant_bounder.h>
#include <jump_detector/least_squares_predictor.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	ros::NodeHandle nh("~");
	
	// Predictor settings
	int order;
	nh.param<int>("predictor_config/order", order, 2);
	
	// Bounder settings
	double bound;
	nh.param<double>("bound_config/bound", bound, 1);
	
	// Ja filter settings
	int max_window_length;
	nh.param<int>("ja_filter_config/max_window_length", max_window_length, 20);
	
	// Initialize jump detector
	LeastSquaresPredictor predictor(order);
	ConstantBounder bounder(bound);
	JumpAwareFilter jafilter(max_window_length, predictor, bounder);

	// Start node
	ExternalForceJumpDetectorNode node(nh, jafilter);
	node.run();
	
	return 0;
}
