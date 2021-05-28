#!/bin/bash

if [ "$#" -ne 1 ]
then
	echo "Usage: ./replay_trajectory.sh <rosbag file>"
else
	rosbag play "$1" /cartesian_pose:=/equilibrium_pose --topics /cartesian_pose
fi
