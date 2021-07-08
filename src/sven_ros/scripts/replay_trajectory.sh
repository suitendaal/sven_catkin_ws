#!/bin/bash

if [ "$#" -eq 1 ]
then
	rosbag play "$1" /cartesian_pose:=/equilibrium_pose --topics /cartesian_pose
elif [ "$#" -eq 2 ]
then
	rosbag play --rate="$2" "$1" /cartesian_pose:=/equilibrium_pose --topics /cartesian_pose
else
	echo "Usage: ./replay_trajectory.sh <rosbag file> [<rate>]"
fi
