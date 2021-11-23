#include <sven_ros/external_force_jump_detector_node.h>
#include <jump_detector_new/impact_aware_force_filter.h>
#include <jump_detector_new/force_rate_bounder.h>
#include <jump_detector_new/force_rate_predictor.h>
#include <string>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	ros::NodeHandle nh("~");
	
	// Predictor settings
	double time_window;
	nh.param<double>("/force_rate_jump_detector/predictor_config/time_window", time_window, 0.035);
	
	// Bounder settings
	double bound;
	nh.param<double>("/force_rate_jump_detector/bounder_config/bound", bound, 1.0 / 0.001);
	
	// Ja filter settings
	int max_window_length;
	nh.param<int>("/force_rate_jump_detector/ja_filter_config/max_window_length", max_window_length, 2);
	
	// Initialize jump detector
	ForceRatePredictor predictor(time_window);
	ForceRateBounder bounder(bound);
	ImpactAwareForceFilter jafilter(max_window_length, predictor, bounder);

	// Start node
	ExternalForceJumpDetectorNode node(nh, jafilter);
	node.run();
	
	return 0;
}
