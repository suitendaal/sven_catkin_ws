#ifndef SVEN_ROS_CONTROLLER_NODE_H
#define SVEN_ROS_CONTROLLER_NODE_H

#include <sven_ros/baseline_controller_node.h>
#include <sven_ros/BoolStamped.h>

class SvenRosControllerNode : public BaselineControllerNode {

protected:

	// Publishers and subscribers
	ros::Subscriber imp_sub_;

	// Configuration
	franka_custom_controllers::ControlOptions control_options_impact_;
	unsigned int interim_ending_mode_;
	double interim_threshold_;
	
	// Impact detection
	bool in_interim_phase_;
	double last_impact_time_;
	double first_interim_impact_time_;
	
	virtual void initialize(std::string reference_trajectory_file) {
/*		BaselineControllerNode::initialize(reference_trajectory_file);*/
		
		imp_sub_ = nh.subscribe("/sven_ros/jump_detector", 20, &SvenRosControllerNode::impact_detector_callback, this, ros::TransportHints().reliable().tcpNoDelay());
		
		// Read configuration
		bool use_position_feedback;
		nh.param<bool>("config/impact_config/use_position_feedback", use_position_feedback, true);
		control_options_impact_.use_position_feedback = use_position_feedback;
		
		bool use_velocity_feedback;
		nh.param<bool>("config/impact_config/use_velocity_feedback", use_velocity_feedback, true);
		control_options_impact_.use_velocity_feedback = use_velocity_feedback;
		
		bool use_velocity_feedforward;
		nh.param<bool>("config/impact_config/use_velocity_feedforward", use_velocity_feedforward, false);
		control_options_impact_.use_velocity_feedforward = use_velocity_feedforward;
		
		bool use_effort_feedforward;
		nh.param<bool>("config/impact_config/use_effort_feedforward", use_effort_feedforward, false);
		control_options_impact_.use_effort_feedforward = use_effort_feedforward;
		
		int stiffness_type;
		nh.param<int>("config/impact_config/stiffness_type", stiffness_type, 1);
		control_options_impact_.stiffness_type = stiffness_type;
		
		bool use_torque_saturation;
		nh.param<bool>("config/impact_config/use_torque_saturation", use_torque_saturation, true);
		control_options_impact_.use_torque_saturation = use_torque_saturation;
		
		double delta_tau_max;
		nh.param<double>("config/impact_config/delta_tau_max", delta_tau_max, 1.0);
		control_options_impact_.delta_tau_max = delta_tau_max;
		
		int interim_ending_mode;
		nh.param<int>("config/interim_config/interim_ending_mode", interim_ending_mode, 1);
		interim_ending_mode_ = interim_ending_mode;
		
		nh.param<double>("config/interim_config/interim_threshold", interim_threshold_, 0.5);
	}
	
	virtual void impact_detector_callback(const sven_ros::BoolStampedPtr &msg) {
		if (msg->data) {
			double time = (ros::Time::now() - starting_time_).toSec();
			ROS_INFO_STREAM("Impact detected at time " << time);
			last_impact_time_ = time;
		}
	}
	
	virtual RobotCommand evaluate(double time) {
		RobotCommand result = BaselineControllerNode::evaluate(time);
		
		if (in_interim_phase_) {
			// Take effort from next phase
			int index = phase_trajectory_times_[current_phase_+1].size() - 1;
			for (int i = 1; i < phase_trajectory_times_[current_phase_+1].size(); i++) {
				if (time < phase_trajectory_times_[current_phase_+1][i]) {
					index = i - 1;
					break;
				}
			}
			
			result.effort = phase_trajectory_data_[current_phase_+1][index].effort;
		}
		
		return result;
	}
	
	virtual void update_phase(double time) {
		// Switch to next phase when current phase has ended
		if (time > phase_trajectory_times_[current_phase_].back()) {
			current_phase_++;
			current_index_ = 0;
			first_interim_impact_time_ = -1;
			in_interim_phase_ = false;
			ROS_INFO_STREAM("End of phase " << current_phase_-1 << " reached at time " << time);
		}
		
		// Interim phase is possible
		if (current_phase_ < phase_trajectory_times_.size() - 1) {
		
			double double_trajectory_starting_time = phase_trajectory_times_[current_phase_+1][0];
			
			// In region of interim phase
			if (time >= double_trajectory_starting_time) {
				
				// Set first impact time
				if (last_impact_time_ > double_trajectory_starting_time && first_interim_impact_time_ < 0) {
					first_interim_impact_time_ = last_impact_time_;
					ROS_INFO_STREAM("Interim phase after phase " << current_phase_ << " started at time " << time);
				}
				
				// In interim phase
				if (first_interim_impact_time_ > double_trajectory_starting_time) {
				
					bool interim_phase_ended;
					
					// Pick interim ending mode
					switch (interim_ending_mode_) {
						case 1:
							// Extend window only after first impact
							interim_phase_ended = time - first_interim_impact_time_ > interim_threshold_;
							break;
						case 0:
						default:
							// Extend window after each impact
							interim_phase_ended = time - last_impact_time_ > interim_threshold_;
							break;
					}
					
					if (!interim_phase_ended) {
						in_interim_phase_ = true;
					} else {
						in_interim_phase_ = false;
						current_phase_++;
						current_index_ = 0;
						first_interim_impact_time_ = -1;
						ROS_INFO_STREAM("End of interim phase after phase " << current_phase_-1 << " reached at time " << time);
					}
				}
			}
		}
	}
	
	virtual void send_options_msg(double time) {
		if (in_interim_phase_) {
			franka_custom_controllers::ControlOptions msg = control_options_impact_;
			msg.phase = 0;
			msg.header.stamp = starting_time_ + ros::Duration(time);
			options_pub_.publish(msg);
		} else {
			BaselineControllerNode::send_options_msg(time);
		}
	}

public:

	SvenRosControllerNode(std::string reference_trajectory_file, double publish_rate)
	: BaselineControllerNode(reference_trajectory_file, publish_rate),
	last_impact_time_(-1),
	first_interim_impact_time_(-1),
	in_interim_phase_(false)
	{
		initialize(reference_trajectory_file);
	}
	
	SvenRosControllerNode(ros::NodeHandle nh, std::string reference_trajectory_file, double publish_rate)
	: BaselineControllerNode(nh, reference_trajectory_file, publish_rate),
	last_impact_time_(-1),
	first_interim_impact_time_(-1),
	in_interim_phase_(false)
	{
		initialize(reference_trajectory_file);
	}

};


#endif // SVEN_ROS_CONTROLLER_NODE_H
