#!/usr/bin/python3

import sys
import pickle
import matplotlib.pyplot as plt
from statistics import mean
import config.config_jump_detection as config
from readers import *
from datalib import *
from filters import *

local_maxima_count = 4
window_lengths = range(3,41)
scores = []
force_ext = []

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

for window_length in window_lengths:

	config.jump_detector.max_window_length = window_length
	config.jump_detector.bounder = NoneBounder()

	# Initialize results
	predictions = []
	all_local_maxima = []
	proposed_bounds = []
	bound_count = dict()

	for i in range(len(config.demos)):
	
		predictions.append(DataSet())
			
#		print("Detecting jumps")
		
		# Detect jumps
		jump_detector = config.jump_detector
		jump_detector.reset()
		for j in range(len(force_ext[i])):
			jump_detected, info = jump_detector.update(force_ext[i][j])
			predictions[i].append(info[0])
			
		#### Output the data
		
		pred_diff = abs(force_ext[i] - predictions[i]).value
		local_maxima = []
		for i in range(len(pred_diff) - 2):
			j = i + 1
			if pred_diff[j] is None:
				continue
			if (pred_diff[j-1] is None or pred_diff[j] > pred_diff[j-1]) and (pred_diff[j+1] is None or pred_diff[j] > pred_diff[j+1]):
				local_maxima.append(pred_diff[j])
		local_maxima.sort(reverse=True)
		all_local_maxima.append(local_maxima)
		proposed_bounds.append(local_maxima[local_maxima_count-1])
		
		# Print result
#		print(f"For demo {demo}, the largest local_maxima are {local_maxima[0:local_maxima_count]}")
		
#	all_local_maxima.sort()
#	print(f"All local maxima are {all_local_maxima} with a minimum value of {min(all_local_maxima)}")

	score = 0
	proposed_bound = max(proposed_bounds)
	config.jump_detector.bounder = ConstantBounder(bound=proposed_bound)
	for i in range(len(config.demos)):
			
		detected_jumps = 0		
			
		# Detect jumps
		jump_detector = config.jump_detector
		jump_detector.reset()
		for j in range(len(force_ext[i])):
			jump_detected, info = jump_detector.update(force_ext[i][j])
			if jump_detected:
				detected_jumps += 1

		score += (local_maxima_count-1-detected_jumps)**2
		
	scores.append(score)
	
	print(f"The proposed bounds per demo for window_length {window_length} are {proposed_bounds} with a maximum value of {max(proposed_bounds)}")
	print(f"Score for window_length {window_length} is {score}")

fig = plt.figure(figsize=config.figsize, dpi=config.dpi)
plt.rcParams['xtick.labelsize'] = config.fontsize2
plt.rcParams['ytick.labelsize'] = config.fontsize2
plt.plot(window_lengths, scores)
plt.title('Score per max window length','-*',fontsize=config.fontsize1)
plt.xlabel('Window length',fontsize=config.fontsize2)
plt.ylabel('Score',fontsize=config.fontsize2)
plt.show()
