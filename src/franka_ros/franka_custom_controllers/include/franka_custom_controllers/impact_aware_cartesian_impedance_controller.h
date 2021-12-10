// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
/*#include <geometry_msgs/PoseStamped.h>*/
#include <std_msgs/Int32.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_hw/trigger_rate.h>
#include <franka_custom_controllers/compliance_paramConfig.h>
#include <franka_custom_controllers/ControlOptions.h>
#include <franka_custom_controllers/RobotState.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_msgs/FrankaState.h>

namespace franka_custom_controllers {

class ImpactAwareCartesianImpedanceController : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // Trigger publish to publish at the correct rate
  franka_hw::TriggerRate trigger_publish_;

  // Stiffness parameters
  double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_impact_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_impact_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  double filter_params_effort_{0.005};
  double effort_stiffness_{0};
  double effort_stiffness_target_{0};
  
  // State parameters
  Eigen::Vector3d position_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Vector3d velocity_d_;
  Eigen::Vector3d velocity_d_target_;
  Eigen::Vector3d rotational_velocity_d_;
  Eigen::Vector3d rotational_velocity_d_target_;
  Eigen::Vector3d acceleration_d_;
  Eigen::Vector3d acceleration_d_target_;
  Eigen::Vector3d rotational_acceleration_d_;
  Eigen::Vector3d rotational_acceleration_d_target_;
  Eigen::Matrix<double, 6, 1> effort_d_;
  Eigen::Matrix<double, 6, 1> effort_d_target_;
  
  // Impact state publisher
  ros::Publisher pub_state_;
  unsigned int sequence_{0};

  // Dynamic stiffness reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_custom_controllers::compliance_paramConfig>> dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_custom_controllers::compliance_paramConfig& config, uint32_t level);
  
  // Control options subscriber
  ros::Subscriber sub_options_;
  void controlOptionsCallback(const franka_custom_controllers::ControlOptionsPtr& msg);
  franka_custom_controllers::ControlOptions control_options_;
  franka_custom_controllers::RobotState command_;
  franka_msgs::FrankaState state_;

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const franka_custom_controllers::RobotStateConstPtr& msg);
  
  // Franka state subscriber
  ros::Subscriber sub_franka_state_;
  void frankaStateCallback(const franka_msgs::FrankaStatePtr& msg);
  
  // Calculate joint limiting torque
  Eigen::Matrix<double, 7, 1> calculateJointLimit(const Eigen::Matrix<double, 7, 1>& q);
};

}  // namespace franka_custom_controllers
