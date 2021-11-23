#ifndef JOINT_JUMP_DETECTOR_NODE_H
#define JOINT_JUMP_DETECTOR_NODE_H

#include <ros/ros.h>

#include "jump_detector/jump_detector.h"
#include <sven_ros/jump_detector_node.h>

#include <sensor_msgs/JointState.h>

class JointJumpDetectorNode : public JumpDetectorNode {

	protected:
		ros::Subscriber joint_data_sub;
		int joint_;
		void joint_state_received(const sensor_msgs::JointStateConstPtr &msg);
		
	public:
		JointJumpDetectorNode(int joint, JumpDetector &detector);
		JointJumpDetectorNode(ros::NodeHandle nh, int joint, JumpDetector &detector);
		~JointJumpDetectorNode();
		void run();
};

#endif // JOINT_JUMP_DETECTOR_NODE_H
