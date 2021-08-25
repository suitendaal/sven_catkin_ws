#include <sven_ros/joint_jump_detector_node.h>
#include <jump_detector/jump_aware_filter.h>
#include <jump_detector/constant_bounder.h>
#include <jump_detector/least_squares_predictor.h>

JointJumpDetectorNode::JointJumpDetectorNode(int joint, JumpDetector &detector)
: joint_(joint),
JumpDetectorNode(detector)
{
  joint_data_sub = nh.subscribe("/franka_state_controller/joint_states", 1000, &JointJumpDetectorNode::joint_state_received, this);
}

JointJumpDetectorNode::JointJumpDetectorNode(ros::NodeHandle nh, int joint, JumpDetector &detector)
: joint_(joint),
JumpDetectorNode(nh, detector)
{
  joint_data_sub = nh.subscribe("/franka_state_controller/joint_states", 1000, &JointJumpDetectorNode::joint_state_received, this);
}

JointJumpDetectorNode::~JointJumpDetectorNode() {
	return;
}

void JointJumpDetectorNode::joint_state_received(const sensor_msgs::JointStateConstPtr &msg) {
  double value = msg->position[joint_-1];
	double time = msg->header.stamp.toSec();
	
	ROS_DEBUG_STREAM("Joint data received from joint " << this->joint_ << " at time " << time << " with value " << value);
	
	bool jump_detected = this->detector_->update(time, value);
	
	if (jump_detected) {
	  ROS_INFO_STREAM("Jump detected at joint " << this->joint_ << " at time " << time);
	}
	
	this->send_jump_detected_msg(jump_detected);
}

void JointJumpDetectorNode::run() {
  ROS_INFO_STREAM("Start jump_detector" << this->joint_);
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
	JointJumpDetectorNode node(nh, joint, jafilter);
	node.run();
	
	return 0;
}
