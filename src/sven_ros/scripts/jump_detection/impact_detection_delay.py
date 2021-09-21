#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from models import *
import config.config_impact_detection_delay as config
import time as t

start_time = t.time()

position_datasets = []
velocity_datasets = []

print("Reading demonstration data")

for demo in config.demos:
	franka_reader = FrankaStateReader(demo)

	position_dataset = PositionDataSet()
	velocity_dataset = PositionDataSet()

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		position_dataset.append(PositionDataPoint(time, dp.position))
		velocity_dataset.append(PositionDataPoint(time, dp.velocity))
		
	position_dataset.align_time()
	velocity_dataset.align_time()
		
	position_datasets.append(position_dataset)
	velocity_datasets.append(velocity_dataset)

print("--- %s seconds ---" % (t.time() - start_time))
print("Analyzing demos")

detection_delays = []

for i in range(len(velocity_datasets)):
	velocity_data = velocity_datasets[i][:(config.impact_intervals[i][0][0]+1)]
	impact_time = velocity_data[-1].time
	for j in range(len(velocity_data)-2):
		index = len(velocity_data) - 1 - j - 1
		if (velocity_data[index].z >= velocity_data[index+1].z and velocity_data[index].z >= velocity_data[index-1].z) or (velocity_data[index].z <= velocity_data[index+1].z and velocity_data[index].z <= velocity_data[index-1].z):
			detection_delays.append(impact_time - velocity_data[index].time)
			break
	if len(detection_delays) < i+1:
		detection_delays.append(0)

print("Detection delays:",detection_delays)
print("Maximal detection delay:",max(detection_delays))
print("--- %s seconds ---" % (t.time() - start_time))
print("Done")
