#ifndef BASELINE_CONTROLLER_NODE_H
#define BASELINE_CONTROLLER_NODE_H

#include <string>
#include <fstream>
#include <ros/ros.h>

#include <franka_custom_controllers/ControlOptions.h>
#include <franka_custom_controllers/RobotState.h>

#include "impedance_control/robot_command.h"

class BaselineControllerNode {

protected:
	
	// Publishers and subscribers
	ros::Publisher pose_pub_;
	ros::Publisher options_pub_;
	
	// Trajectory data
	std::vector<std::vector<double>> phase_trajectory_times_;
	std::vector<std::vector<RobotCommand>> phase_trajectory_data_;
	unsigned int current_index_;
	unsigned int current_phase_;
	
	// Configuration
	franka_custom_controllers::ControlOptions control_options_;
	std::vector<bool> phase_use_effort_;
	
	// Time
	ros::Time starting_time_;
	ros::Rate loop_rate_;
	
	virtual void read_reference_trajectory(std::string reference_trajectory_file) {
		ROS_INFO_STREAM("Trajectory file: " << reference_trajectory_file);
		
		std::ifstream stream(reference_trajectory_file);
		
		if (!stream)
		{
			ROS_INFO_STREAM("Something went wrong opening file " << reference_trajectory_file);
		} 
		
		std::string line;
		while(std::getline(stream, line)) {
		
			RobotCommand command;
			double time;
			unsigned int phase;
			
			unsigned int index = 0;
			
			std::istringstream s(line);
			std::string field;
			while (getline(s, field, ',')) {
				switch (index) {
					case 0:
						phase = std::stoi(field);
						break;
					case 1:
						time = std::stod(field);
						break;
					case 2:
					case 3:
					case 4:
						command.position.push_back(std::stod(field));
						break;
					case 5:
					case 6:
					case 7:
						command.velocity.push_back(std::stod(field));
						break;
					case 8:
					case 9:
					case 10:
					case 11:
						command.orientation.push_back(std::stod(field));
						break;
					case 12:
					case 13:
					case 14:
					case 15:
					case 16:
					case 17:
						command.effort.push_back(std::stod(field));
						break;
					default:
						break;
				}
				
				index++;
			}
			
			while (phase_trajectory_times_.size() < phase+1) {
				phase_trajectory_times_.push_back(std::vector<double>());
				phase_trajectory_data_.push_back(std::vector<RobotCommand>());
			}
			
			phase_trajectory_times_[phase].push_back(time);
			phase_trajectory_data_[phase].push_back(command);
		}
	}
	
	virtual void initialize(std::string reference_trajectory_file) {
		read_reference_trajectory(reference_trajectory_file);
	
		pose_pub_ = nh.advertise<franka_custom_controllers::RobotState>("/equilibrium_state", 1);
		options_pub_ = nh.advertise<franka_custom_controllers::ControlOptions>("/impedance_control_options", 1);
		
		// Read configuration
		bool use_position_feedback;
		nh.param<bool>("config/default_config/use_position_feedback", use_position_feedback, true);
		control_options_.use_position_feedback = use_position_feedback;
		
		bool use_velocity_feedback;
		nh.param<bool>("config/default_config/use_velocity_feedback", use_velocity_feedback, true);
		control_options_.use_velocity_feedback = use_velocity_feedback;
		
		bool use_velocity_feedforward;
		nh.param<bool>("config/default_config/use_velocity_feedforward", use_velocity_feedforward, false);
		control_options_.use_velocity_feedforward = use_velocity_feedforward;
		
		nh.param<std::vector<bool>>("config/default_config/use_effort_feedforward", phase_use_effort_, std::vector<bool>());
		
		int stiffness_type;
		nh.param<int>("config/default_config/stiffness_type", stiffness_type, 0);
		control_options_.stiffness_type = stiffness_type;
		
		bool use_torque_saturation;
		nh.param<bool>("config/default_config/use_torque_saturation", use_torque_saturation, true);
		control_options_.use_torque_saturation = use_torque_saturation;
		
		double delta_tau_max;
		nh.param<double>("config/default_config/delta_tau_max", delta_tau_max, 1.0);
		control_options_.delta_tau_max = delta_tau_max;
	}
	
	virtual RobotCommand evaluate(double time) {
		int index = phase_trajectory_times_[current_phase_].size() - 1;
		for (int i = current_index_ + 1; i < phase_trajectory_times_[current_phase_].size(); i++) {
			if (time < phase_trajectory_times_[current_phase_][i]) {
				index = i - 1;
				break;
			}
		}
		current_index_ = index;
		return phase_trajectory_data_[current_phase_][current_index_];
	}
	
	virtual void update_phase(double time) {
		// Switch to next phase when current phase has ended
		if (time > phase_trajectory_times_[current_phase_].back()) {
			current_phase_++;
			current_index_ = 0;
			if (!has_ended(time)) {
				ROS_INFO_STREAM("Updating to phase " << current_phase_ << " at time " << time);
			}
		}
	}
	
	virtual void send_options_msg(double time) {
		franka_custom_controllers::ControlOptions msg = control_options_;
		msg.header.stamp = starting_time_ + ros::Duration(time);
		msg.use_effort_feedforward = phase_use_effort_[current_phase_];
		msg.phase = current_phase_ + 1;
		options_pub_.publish(msg);
	}
	
	virtual void send_pose_msg(double time) {
		RobotCommand command = evaluate(time);
		franka_custom_controllers::RobotState msg;
		msg.header.stamp = starting_time_ + ros::Duration(time);
		msg.pose.position.x = command.position[0];
		msg.pose.position.y = command.position[1];
		msg.pose.position.z = command.position[2];
		msg.pose.orientation.x = command.orientation[0];
		msg.pose.orientation.y = command.orientation[1];
		msg.pose.orientation.z = command.orientation[2];
		msg.pose.orientation.w = command.orientation[3];
		msg.velocity.linear.x = command.velocity[0];
		msg.velocity.linear.y = command.velocity[1];
		msg.velocity.linear.z = command.velocity[2];
		msg.effort.linear.x = command.effort[0];
		msg.effort.linear.y = command.effort[1];
		msg.effort.linear.z = command.effort[2];
		msg.effort.angular.x = command.effort[3];
		msg.effort.angular.y = command.effort[4];
		msg.effort.angular.z = command.effort[5];
		pose_pub_.publish(msg);
	}
	
	virtual bool has_started(double time) {
		return (phase_trajectory_times_.size() > 0 && time >= phase_trajectory_times_[0][0]);
	}
	
	virtual bool has_ended(double time) {
		return current_phase_ >= phase_trajectory_times_.size();
	}
	
public:
	ros::NodeHandle nh;

	BaselineControllerNode(std::string reference_trajectory_file, double publish_rate)
	: current_index_(0),
	current_phase_(0),
	nh(),
	loop_rate_(publish_rate)
	{
		initialize(reference_trajectory_file);
	}

	BaselineControllerNode(ros::NodeHandle nh, std::string reference_trajectory_file, double publish_rate)
	: current_index_(0),
	current_phase_(0),
	nh(nh),
	loop_rate_(publish_rate)
	{
		initialize(reference_trajectory_file);
	}

	~BaselineControllerNode() {
		return;
	}

	virtual int run() {
		starting_time_ = ros::Time::now();
		ROS_INFO_STREAM("Starting trajectory");
		
		while (ros::ok()) {
			double time = (ros::Time::now() - starting_time_).toSec();
			
			if (has_started(time)) {
				update_phase(time);
				
				if (has_ended(time)) {
					ROS_INFO_STREAM("Ended at time " << time);
					return 0;
				}
				
				send_options_msg(time);
				send_pose_msg(time);
			}
			
			else if (has_ended(time)) {
				ROS_INFO_STREAM("Something went wrong at time " << time);
				return 1;
			}
			
			loop_rate_.sleep();
		}
	}
};


#endif // BASELINE_CONTROLLER_NODE_H
