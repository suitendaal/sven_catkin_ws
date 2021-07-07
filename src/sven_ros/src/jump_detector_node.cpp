#include "sven_ros/jump_detector_node.h"
#include "jump_detector/jump_aware_filter.h"
#include "jump_detector/constant_bounder.h"
#include "jump_detector/weighted_predictor.h"
#include "jump_detector/polynomial.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	
	WeightedPredictor predictor(3);
	ConstantBounder bounder(0.4);
	JumpAwareFilter jafilter(20, predictor, bounder);
	
	std::vector<DataPoint> data;
	for (int i = 0; i < 10; i++) {
		DataPoint point(i, 10*i);
		data.push_back(point);
	}
	
	double value;
	double value2;
	
	bool result = bounder.bound(data, 0.1, value);
	bool result2 = predictor.predict(data, 0.1, value2);
	
	ROS_INFO_STREAM("Bounder test: " << result << ", " << value);
	ROS_INFO_STREAM("Predictor test: " << result2 << ", " << value2);
	
	for (int i = 0; i < 10; i++) {
		ROS_INFO_STREAM("Ja filter test: " << jafilter.datapoint_arrived(i, 10*i) << ", " << jafilter.get_current_window_length());
	}
	for (int i = 0; i < 10; i++) {
		ROS_INFO_STREAM("Ja filter test: " << jafilter.datapoint_arrived(i+10, 10*i) << ", " << jafilter.get_current_window_length());
	}
	
	std::vector<double> times = { 0, 1, 2, 3, 4, 5 };
	std::vector<double> values = { 2, 3, 4, 5, 4, 3 };
	int order = 3;
	std::vector<double> coefs = polynomial::polyfit(times, values, order);
	
	std::cout << "Coefs: ";
	for (int i = 0; i < coefs.size(); i++) {
		std::cout << coefs[i] << ", ";
	}
	std::cout << std::endl;
	
	std::cout << "Result: ";
	for (int i = 0; i < times.size(); i++) {
		std::cout << values[i] << ": " << polynomial::polyval(coefs, times[i]) << ", ";
	}
	std::cout << std::endl;
	
	return 0;
}
