#!/bin/bash

rosbag record /joint_states /cartesian_pose /franka_state_controller/franka_states /vive/pose3 /equilibrium_pose /impact_aware_cartesian_impedance_controller/impact_control_state

