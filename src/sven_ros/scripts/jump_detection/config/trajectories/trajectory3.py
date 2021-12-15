#!/usr/bin/python3

from datalib import *
import math

height = 0.1
bottom = -0.15
velocity = 0.15

table_height = 0
hitting_y = 0.4
angle = math.pi / 3

starting_y = -(height-table_height) / math.tan(angle) + hitting_y
ending_y = (table_height - bottom) / math.tan(angle) + hitting_y
y_velocity = velocity * math.cos(angle)

starting_time = 2
hitting_time = starting_time + (hitting_y - starting_y) / y_velocity
ending_time = starting_time + (ending_y - starting_y) / y_velocity
hitting_height = height - velocity * math.sin(angle) * (hitting_time - starting_time)
additional_sliding_time = 0.5

x = 0.3

via_points = PoseDataSet([
	PoseDataPoint(0, [x, starting_y, height, math.pi, 0, math.pi]),
	PoseDataPoint(starting_time, [x, starting_y, height, math.pi, 0, math.pi]),
	PoseDataPoint(hitting_time, [x, hitting_y, hitting_height, math.pi, 0, 0]),
	PoseDataPoint(ending_time, [x, ending_y, bottom, math.pi, 0, 0]),
	PoseDataPoint(ending_time + additional_sliding_time, [x, ending_y + additional_sliding_time * y_velocity, bottom, math.pi, 0, 0])
])
