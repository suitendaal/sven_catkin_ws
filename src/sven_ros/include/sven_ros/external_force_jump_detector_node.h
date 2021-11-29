#ifndef EXTERNAL_FORCE_JUMP_DETECTOR_NODE_H
#define EXTERNAL_FORCE_JUMP_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sven_ros/franka_state_jump_detector_node.h>

// For debugging
#include <jump_detector_new/impact_aware_force_filter.h>
#include <memory>
#include <iostream>
#include <iomanip>

namespace external_force_functions {
	double magnitude(std::vector<double> force_vector) {
		double result = 0;
		for (int i = 0; i < force_vector.size(); i++) {
			result += force_vector[i] * force_vector[i];
		}
		return sqrt(result);
	}
};

class ExternalForceJumpDetectorNode : public FrankaStateJumpDetectorNode {

	protected:
		void franka_state_received(const franka_msgs::FrankaStateConstPtr &msg){
			double time = msg->header.stamp.toSec();
			const double* data = msg->O_F_ext_hat_K.data();
			std::vector<double> force_vector(data, data + 3);
/*			double force = external_force_functions::magnitude(force_vector);*/
			
/*			ROS_DEBUG_STREAM("External force data received at time " << time << " with value " << force);*/
	
			bool jump_detected = this->detector_->update(time, force_vector);
			
			if (jump_detected) {
				ROS_INFO_STREAM(std::fixed << std::setprecision(4) << "Jump detected at time " << time << " with force value: ||[" << force_vector[0] << ", " << force_vector[1] << ", " << force_vector[2] << "]||");//, = " << force);
			}
			
			this->send_jump_detected_msg(msg->header.stamp, jump_detected);
		}
		
	public:
		ExternalForceJumpDetectorNode(JumpDetector &detector)
		: FrankaStateJumpDetectorNode(detector)
		{}
		
		ExternalForceJumpDetectorNode(ros::NodeHandle nh, JumpDetector &detector)
		: FrankaStateJumpDetectorNode(nh, detector)
		{}
		
		virtual void run() {
			ROS_INFO_STREAM("Start external force jump_detector");
	ros::spin();
		}
};

#endif // EXTERNAL_FORCE_JUMP_DETECTOR_NODE_H
