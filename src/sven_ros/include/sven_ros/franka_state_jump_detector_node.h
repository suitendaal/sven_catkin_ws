#ifndef FRANKA_STATE_JUMP_DETECTOR_NODE_H
#define FRANKA_STATE_JUMP_DETECTOR_NODE_H

#include <ros/ros.h>

#include "jump_detector_new/jump_detector.h"
#include <sven_ros/jump_detector_node.h>

#include <franka_msgs/FrankaState.h>

class FrankaStateJumpDetectorNode : public JumpDetectorNode {

	protected:
		ros::Subscriber data_sub_;
		virtual void franka_state_received(const franka_msgs::FrankaStateConstPtr &msg) = 0;
		
	public:
		FrankaStateJumpDetectorNode(JumpDetector &detector)
		: JumpDetectorNode(detector)
		{
			data_sub_ = nh.subscribe("/franka_state_controller/franka_states", 1, &FrankaStateJumpDetectorNode::franka_state_received, this);
		}
		
		FrankaStateJumpDetectorNode(ros::NodeHandle nh, JumpDetector &detector)
		: JumpDetectorNode(nh, detector)
		{
			data_sub_ = nh.subscribe("/franka_state_controller/franka_states", 1, &FrankaStateJumpDetectorNode::franka_state_received, this);
		}
		
		virtual void run() {
			ROS_INFO_STREAM("Start franka_state jump_detector");
	ros::spin();
		}
};

#endif // FRANKA_STATE_JUMP_DETECTOR_NODE_H
