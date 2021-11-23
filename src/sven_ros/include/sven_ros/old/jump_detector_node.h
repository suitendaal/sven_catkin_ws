#ifndef JUMP_DETECTOR_NODE_H
#define JUMP_DETECTOR_NODE_H

#include <ros/ros.h>

#include "jump_detector/jump_detector.h"

#include <sven_ros/BoolStamped.h>

class JumpDetectorNode {

	protected:
		JumpDetector* detector_;
		ros::Publisher jump_detector_pub_;
		unsigned int sequence_;
	
		void send_jump_detected_msg(bool jump_detected) {
			sven_ros::BoolStamped msg_out;
			
			// Define header
			std_msgs::Header header;
			header.seq = this->sequence_;
			header.stamp = ros::Time::now();
			header.frame_id = "Jump Detector";
			msg_out.header = header;
			
			// Define data
			msg_out.data = jump_detected;
			
			// Send message
			jump_detector_pub_.publish(msg_out);
			
			// Update sequence
			this->sequence_++;
		}
	
		
	public:
		ros::NodeHandle nh;
		
		JumpDetectorNode(JumpDetector &detector)
		: detector_(&detector),
		nh()
		{
			jump_detector_pub_ = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 1000);
		}
		
		JumpDetectorNode(ros::NodeHandle nh, JumpDetector &detector)
		: detector_(&detector),
		nh(nh)
		{
			jump_detector_pub_ = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 1000);
		}
		
		~JumpDetectorNode(){
			return;
		}
		
		virtual void run() = 0;

};

#endif // JUMP_DETECTOR_NODE_H
