#include <sven_ros/jump_detector_node.h>
#include <jump_detector/jump_aware_filter.h>
#include <jump_detector/constant_bounder.h>
#include <jump_detector/weighted_predictor.h>

JumpDetectorNode::JumpDetectorNode(int joint, JumpDetector &detector)
: joint_(joint),
detector_(&detector),
nh()
{
  jump_detector_pub = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 1000);
	joint_data_sub = nh.subscribe("/franka_state_controller/joint_states", 1000, &JumpDetectorNode::joint_state_received, this);
}

JumpDetectorNode::JumpDetectorNode(ros::NodeHandle nh, int joint, JumpDetector &detector)
: nh(nh),
joint_(joint),
detector_(&detector)
{
  jump_detector_pub = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 1000);
	joint_data_sub = nh.subscribe("/franka_state_controller/joint_states", 1000, &JumpDetectorNode::joint_state_received, this);
}

JumpDetectorNode::~JumpDetectorNode() {
  return;
}

void JumpDetectorNode::joint_state_received(const sensor_msgs::JointStateConstPtr &msg) {
  double value = msg->position[joint_-1];
	double time = msg->header.stamp.toSec();
	
	ROS_DEBUG_STREAM("Joint data received from joint " << joint_ << " at time " << time << " with value " << value);
	
	bool jump_detected = detector_->datapoint_arrived(time, value);
	
	if (jump_detected) {
	  ROS_INFO_STREAM("Jump detected at joint " << joint_ << " at time " << time);
	}
	
	sven_ros::BoolStamped msg_out;
	msg_out.header = msg->header;
	msg_out.data = jump_detected;
	
	jump_detector_pub.publish(msg_out);
}

void JumpDetectorNode::run() {
  ROS_INFO_STREAM("Start jump_detector" << joint_);
	ros::spin();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "jump_detector_node");
	ros::NodeHandle nh("~");
	
	// Get settings from parameter server
	int joint;
	nh.param<int>("joint", joint, 0);
	
	// Predictor settings
	int order;
	nh.param<int>("predictor_config/order", order, 3);
	int n_not_analyzed;
	nh.param<int>("predictor_config/n_not_analyzed", n_not_analyzed, 0);
	int min_n_datapoints;
	nh.param<int>("predictor_config/min_n_datapoints", min_n_datapoints, 3);
	
	// Bounder settings
	double bound;
	nh.param<double>("bound_config/bound", bound, 1);
	
	// Ja filter settings
	int max_window_length;
	nh.param<int>("ja_filter_config/max_window_length", max_window_length, 20);
	
	// Initialize jump detector
	WeightedPredictor predictor(order, n_not_analyzed, min_n_datapoints);
	ConstantBounder bounder(bound);
	JumpAwareFilter jafilter(max_window_length, predictor, bounder);

	// Start node
	JumpDetectorNode node(nh, joint, jafilter);
	node.run();
	
	return 0;
}
