#ifndef JUMP_DETECTOR_NODE_H
#define JUMP_DETECTOR_NODE_H

#include <ros/ros.h>

#include "jump_detector_new/jump_detector.h"

#include <sven_ros/BoolStamped.h>

class JumpDetectorNode {

	protected:
		JumpDetector* detector_;
		ros::Publisher jump_detector_pub_;
	
		void send_jump_detected_msg(ros::Time time, bool jump_detected) {
			sven_ros::BoolStamped msg_out;
			
			// Define header
			std_msgs::Header header;
			header.stamp = time;
			header.frame_id = "JD";
			msg_out.header = header;
			
			// Define data
			msg_out.data = jump_detected;
			
			// Send message
			jump_detector_pub_.publish(msg_out);
		}
	
		
	public:
		ros::NodeHandle nh;
		
		JumpDetectorNode(JumpDetector &detector)
		: detector_(&detector),
		nh()
		{
			jump_detector_pub_ = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 10);
		}
		
		JumpDetectorNode(ros::NodeHandle nh, JumpDetector &detector)
		: detector_(&detector),
		nh(nh)
		{
			jump_detector_pub_ = nh.advertise<sven_ros::BoolStamped>("/sven_ros/jump_detector", 10);
		}
		
		~JumpDetectorNode(){
			return;
		}
		
		virtual void run() = 0;

};

#endif // JUMP_DETECTOR_NODE_H
