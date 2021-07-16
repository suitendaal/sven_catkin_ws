#!/bin/bash

if [ "$#" -ne 3 ]
then
	echo "Usage: ./rosbag2csv.sh <rosbag file> <topic> <csv file>"
else
	rostopic echo -b "$1" -p "$2" > "$3"
fi

