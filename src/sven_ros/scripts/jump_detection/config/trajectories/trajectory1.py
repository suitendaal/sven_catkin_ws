#!/usr/bin/python3

from datalib import *
import math

height = 0.2
table_height = 0
bottom = -0.15
velocity = 0.15
starting_time = 2
hitting_time = starting_time + (height - table_height) / velocity
ending_time = starting_time + (height - bottom) / velocity

via_points = PoseDataSet([
	PoseDataPoint(0, [0, 0.6, height, math.pi, -0.03, math.pi/2]),
	PoseDataPoint(starting_time, [0, 0.6, height, math.pi, -0.03, math.pi/2]),
	PoseDataPoint(ending_time, [0, 0.6, bottom, math.pi, -0.03, math.pi/2]),
	PoseDataPoint(ending_time + starting_time, [0, 0.6, bottom, math.pi, -0.03, math.pi/2])
])
