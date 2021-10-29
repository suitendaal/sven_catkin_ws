// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_custom_controllers/demonstration_cartesian_impedance_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_example_controllers/pseudo_inversion.h>
#include <franka_custom_controllers/DemonstrationControllerState.h>

namespace franka_custom_controllers {

bool DemonstrationCartesianImpedanceController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
  // Equilibrium pose
  sub_equilibrium_pose_ = node_handle.subscribe("/equilibrium_pose", 20, &DemonstrationCartesianImpedanceController::equilibriumPoseCallback, this, ros::TransportHints().reliable().tcpNoDelay());
  
  // Controller state publisher
  pub_state_ = node_handle.advertise<franka_custom_controllers::DemonstrationControllerState>("demonstration_control_state", 20);
  

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Could not read parameter arm_id");
    return false;
  }
  
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("DemonstrationCartesianImpedanceController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Error getting model interface from hardware");
    return false;
  }
  
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } 
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
  } 
  catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Exception getting state handle from interface: " << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } 
    catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("DemonstrationCartesianImpedanceController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ = ros::NodeHandle("dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<dynamic_reconfigure::Server<franka_custom_controllers::compliance_param_demonstrationConfig>>(dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(boost::bind(&DemonstrationCartesianImpedanceController::complianceParamCallback, this, _1, _2));

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  cartesian_stiffness_.setZero();
  cartesian_damping_.setZero();

  return true;
}

void DemonstrationCartesianImpedanceController::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array =
      model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  // convert to eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  // set nullspace equilibrium configuration to initial q
  q_d_nullspace_ = q_initial;
}

void DemonstrationCartesianImpedanceController::update(const ros::Time& time,
                                                 const ros::Duration& /*period*/) {
  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ = filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ = filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ = filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);

  // get state variables
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  std::array<double, 7> gravity_array = model_handle_->getGravity();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  // convert to Eigen
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
      robot_state.tau_J_d.data());
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.linear());
  
  // Get other variables to record
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_external(robot_state.tau_ext_hat_filtered.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());

  // compute error to desired pose
  // position error
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position - position_d_;

  // orientation error
  if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
    orientation.coeffs() << -orientation.coeffs();
  }
  // "difference" quaternion
  Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
  error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
  // Transform to base frame
  error.tail(3) << -transform.linear() * error.tail(3);

  // compute control
  // allocate variables
  Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

  // pseudoinverse for nullspace handling
  // kinematic pseuoinverse
  Eigen::MatrixXd jacobian_transpose_pinv;
  franka_example_controllers::pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

  // TEST
  Eigen::Matrix<double, 6, 1> F_measured = jacobian_transpose_pinv * tau_measured;
  // TEST

  // Cartesian PD control with damping ratio = 1
  Eigen::Matrix<double, 6, 1> F_task = -cartesian_stiffness_ * error - cartesian_damping_ * (jacobian * dq);
  limit_force(F_task);
  tau_task << jacobian.transpose() * F_task;

  // nullspace PD control with damping ratio = 1
  tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) - jacobian.transpose() * jacobian_transpose_pinv) * (nullspace_stiffness_ * (q_d_nullspace_ - q) - (2.0 * sqrt(nullspace_stiffness_)) * dq);
  Eigen::Matrix<double, 7, 1> tau_joint_limit = calculateJointLimit(q);
  // Desired torque
  tau_d << tau_task + tau_nullspace + coriolis + tau_joint_limit;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }
  
  // Send control state message
  DemonstrationControllerState msg;
  msg.header.seq = sequence_;
  msg.header.stamp = time;
  msg.header.frame_id = "DemonstrationCartesianImpedanceController";
  
  for (int i = 0; i < 7; i++) {
    msg.coriolis[i] = coriolis[i];
    msg.gravity[i] = gravity[i];
    msg.q[i] = q.data()[i];
    msg.dq[i] = dq.data()[i];
    msg.tau_external[i] = tau_external.data()[i];
    msg.tau_measured[i] = tau_measured.data()[i];
    msg.tau_task[i] = tau_task[i];
    msg.tau_nullspace[i] = tau_nullspace[i];
    msg.tau_d[i] = tau_d[i];
  }
  
  for (int i = 0; i < 42; i++) {
    msg.jacobian[i] = jacobian_array[i];
  }
  
  for (int i = 0; i < 16; i++) {
    msg.O_T_EE[i] = transform.data()[i];
  }
  
  for (int i = 0; i < 3; i++) {
    msg.position_d[i] = position_d_[i];
    msg.F_task[i] = F_task[i];
    msg.F_task[i+3] = F_task[i+3];
    msg.F_measured[i] = F_measured[i];
    msg.F_measured[i+3] = F_measured[i+3];
  }
  
  msg.orientation_d[0] = orientation_d_.x();
  msg.orientation_d[1] = orientation_d_.y();
  msg.orientation_d[2] = orientation_d_.z();
  msg.orientation_d[3] = orientation_d_.w();
  
  pub_state_.publish(msg);
  sequence_++;
}

Eigen::Matrix<double, 7, 1> DemonstrationCartesianImpedanceController::saturateTorqueRate(const Eigen::Matrix<double, 7, 1>& tau_d_calculated, const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void DemonstrationCartesianImpedanceController::limit_force(Eigen::Matrix<double, 6, 1> &force) {
  for (int i = 0; i < 6; i++) {
  	force(i) = std::max(std::min(force(i), max_cartesian_force_(i)), -max_cartesian_force_(i));
  }
}

void DemonstrationCartesianImpedanceController::complianceParamCallback(
    franka_custom_controllers::compliance_param_demonstrationConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_(0, 0) = config.translational_stiffness_X;
  cartesian_stiffness_target_(1, 1) = config.translational_stiffness_Y;
  cartesian_stiffness_target_(2, 2) = config.translational_stiffness_Z;
//  cartesian_stiffness_target_.topLeftCorner(3, 3)
//      << config.translational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_(0, 0) = 2.0 * sqrt(config.translational_stiffness_X);
  cartesian_damping_target_(1, 1) = 2.0 * sqrt(config.translational_stiffness_Y);
  cartesian_damping_target_(2, 2) = 2.0 * sqrt(config.translational_stiffness_Z);
//  cartesian_damping_target_.topLeftCorner(3, 3)
//      << 2.0 * sqrt(config.translational_stiffness) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
  
  max_cartesian_force_(0) = config.max_force_X;
  max_cartesian_force_(1) = config.max_force_Y;
  max_cartesian_force_(2) = config.max_force_Z;
  for (int i = 3; i < 6; i++) {
  	max_cartesian_force_(i) = config.max_cartesian_torque;
  }
}

void DemonstrationCartesianImpedanceController::equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

Eigen::Matrix<double, 7, 1> DemonstrationCartesianImpedanceController::calculateJointLimit(const Eigen::Matrix<double, 7, 1>& q) {
  // set joint limit in order to avoid to have to controller to go to joint limit
  Eigen::Matrix<double, 7, 1> tau_joint_limit;
  tau_joint_limit.setZero();                      
  if (q(0) > 2.85)     { tau_joint_limit(0) = -2; } 
  else if (q(0) < -2.85)    { tau_joint_limit(0) = +2; }
  if (q(1) > 1.7)      { tau_joint_limit(1) = -2; }
  else if (q(1) < -1.7)     { tau_joint_limit(1) = +2; }
  if (q(2) > 2.85)     { tau_joint_limit(2) = -2; }
  else if (q(2) < -2.85)    { tau_joint_limit(2) = +2; }
  if (q(3) > -0.1)     { tau_joint_limit(3) = -2; }
  else if (q(3) < -3.0)     { tau_joint_limit(3) = +2; }
  if (q(4) > 2.85)     { tau_joint_limit(4) = -2; }
  else if (q(4) < -2.85)    { tau_joint_limit(4) = +2; }
  if (q(5) > 3.7)      { tau_joint_limit(5) = -2; }  
  else if (q(5) < -0.1)     { tau_joint_limit(5) = +2; }
  if (q(6) > 2.8)      { tau_joint_limit(6) = -2; }
  else if (q(6) < -2.8)     { tau_joint_limit(6) = +2; }
  return tau_joint_limit;
}

}  // namespace franka_custom_controllers

PLUGINLIB_EXPORT_CLASS(franka_custom_controllers::DemonstrationCartesianImpedanceController,
                       controller_interface::ControllerBase)
