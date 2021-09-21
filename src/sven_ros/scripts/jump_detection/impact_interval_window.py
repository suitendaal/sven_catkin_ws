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

impact_interval_windows = []

for i in range(len(position_datasets)):
	position_data = position_datasets[i]
	impact_interval_window = []
	for j in config.impact_intervals[i]:
		print(j)
		for k in range(len(j)-1):
			impact_interval_window.append(position_data[j[k+1]].time - position_data[j[k]].time)
	impact_interval_windows.append(impact_interval_window)

print("Impact interval windows:",impact_interval_windows)
print("Maximal impact interval window:",max([max(windows) for windows in impact_interval_windows]))
print("--- %s seconds ---" % (t.time() - start_time))
print("Done")
