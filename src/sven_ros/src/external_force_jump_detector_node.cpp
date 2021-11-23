#include <sven_ros/external_force_jump_detector_node.h>
#include <jump_detector_new/impact_aware_force_filter.h>
#include <jump_detector_new/constant_bounder.h>
#include <jump_detector_new/least_squares_predictor.h>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	ros::NodeHandle nh("~");
	
	// Predictor settings
	int order;
	nh.param<int>("/external_force_jump_detector/predictor_config/order", order, 2);
	
	// Bounder settings
	double bound;
	nh.param<double>("/external_force_jump_detector/bounder_config/bound", bound, 6);
	
	// Ja filter settings
	int max_window_length;
	nh.param<int>("/external_force_jump_detector/ja_filter_config/max_window_length", max_window_length, 20);
	
	// Initialize jump detector
	LeastSquaresPredictor predictor(order);
	ConstantBounder bounder(bound);
	ImpactAwareForceFilter jafilter(max_window_length, predictor, bounder);

	// Start node
	ExternalForceJumpDetectorNode node(nh, jafilter);
	node.run();
	
	return 0;
}
