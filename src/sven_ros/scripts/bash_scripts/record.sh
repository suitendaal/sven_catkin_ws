#!/bin/bash

rosbag record /joint_states /cartesian_pose /franka_state_controller/franka_states /equilibrium_pose /impact_aware_cartesian_impedance_controller/impact_control_state /impedance_control_mode /vive/pose3 /sven_ros/jump_detector /demonstration_cartesian_impedance_controller/demonstration_control_state

