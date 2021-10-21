#!/bin/bash

if [ "$#" -eq 1 ]
then
	rosbag play "$1" --topics /equilibrium_pose
else
	echo "Usage: ./replay_vive_input.sh <rosbag file>"
fi
