#!/usr/bin/python3

import sys
import pickle
import matplotlib.pyplot as plt
from statistics import mean
import config.config_jump_detection as config
from readers import *
from datalib import *
from filters import *

expected_jumps = 2
force_ext = []

# Initialize results
scores = []
predictions = []
jump_indices = []
proposed_bounds = []

for i in range(len(config.demos)):

#		print("Analyzing " + config.demos[i])

	#### Analyze the data
	
	# Initialize results
	force_ext.append(DataSet())

	# Read data
	demo = config.demos[i]
	franka_reader = FrankaStateReader(demo)
	while not franka_reader.end():
		dp = franka_reader.next_datapoint()
		time = dp.time
		franka_state = dp.value
		
		force_ext[i].append(DataPoint(time, franka_state.force_external_magnitude))
	
	# Align time
	force_ext[i].align_time()

	predictions.append(DataSet())
	jump_indices.append([])
		
#		print("Detecting jumps")
	
	# Detect jumps
	jump_detector = config.jump_detector
	jump_detector.reset()
	for j in range(len(force_ext[i])):
		jump_detected, info = jump_detector.update(force_ext[i][j])
		predictions[i].append(info[0])
		if jump_detected:
			jump_indices[i].append(j)
		
	#### Output the data
	score = abs(expected_jumps-len(jump_indices[i]))
	
	scores.append(score)

	print(f"Score for demo {config.demos[i]} is {score}")
	
	# Demo is valid
	if score == 0:
		jump_diffs = DataSet([force_ext[i][j] - predictions[i][j] for j in jump_indices[i]])
		proposed_bounds.append(min(jump_diffs.value))
		
print(f"The proposed bounds are {proposed_bounds} with a minumum value of {min(proposed_bounds)}")

fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
plt.rcParams['xtick.labelsize'] = config.fontsize2
plt.rcParams['ytick.labelsize'] = config.fontsize2
plt.plot(config.demos, scores,'-*')
plt.title('Score per demo',fontsize=config.fontsize1)
plt.xlabel('Demonstration',fontsize=config.fontsize2)
plt.ylabel('Score',fontsize=config.fontsize2)
plt.xticks(rotation=45)

plt.show()
