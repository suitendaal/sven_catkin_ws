#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
import config
from datalib import *

# Apply jump aware filter
for i in range(config.n_joints):
	print("Joint", i+1)
	config.joints[i].detect_jumps()
	
print("Jumps detected")

quit_plot = False
	
# Analyze the jump aware filter behaviour
fontsize1 = 20
fontsize2 = 16
figs = []
for i in range(config.n_joints):
	figs.append(plt.subplots())

for i in range(len(config.demos)):
	for k in range(len(config.joints[0].joint_data[i].pos_data)):
	
		joint_jumping_indexes = []
		
		for j in range(config.n_joints):
			joint_data = config.joints[j].joint_data[i]
			jumping_indexes = joint_data.jumping_indexes
			joint_jumping_indexes.extend(jumping_indexes)
			
		if k >= min(joint_jumping_indexes) - 10 and k <= max(joint_jumping_indexes) + 10:	
	
			for j in range(config.n_joints):
				joint_data = config.joints[j].joint_data[i]
				pos_data = joint_data.pos_data
				jumping_indexes = joint_data.jumping_indexes
				[predictions, bounds, prediction_functions, bound_functions, window_lengths] = joint_data.info
				
				pred_fun = prediction_functions[k]
				bound_fun = bound_functions[k]
				window_length = window_lengths[k]
				start = max(0, k - window_length)
				end = k + 1
				data = pos_data[start:end]
				
				pred_fun_values = []
				bound_fun_values = []
				for l in data:
					pred_fun_values.append(pred_fun.evaluate(time=l.time).value[0])
					bound_fun_values.append(bound_fun.evaluate(time=l.time).value[0])
				jump_points = DataSet(timefactor=pos_data.timefactor)
				for l in jumping_indexes:
					if l >= start and l < end:
						jump_points.append(pos_data[l])
				
				times = data.time()
				data_values = data.values()
				
				# Axis.plot
				figs[j][1].clear()
				figs[j][1].plot(times.copy(), data_values.copy(), '-*',linewidth=2,markersize=10)
				figs[j][1].plot(times.copy(), pred_fun_values.copy(), '-*',linewidth=1,markersize=10)
				figs[j][1].plot(jump_points.time().copy(), jump_points.values().copy(), '*',markersize=12)
				figs[j][1].legend(['Data','Prediction','Jumps'],fontsize=fontsize2)
				figs[j][1].set_xlabel('Time [s]',fontsize=fontsize2)
				figs[j][1].set_ylabel('Position [rad]',fontsize=fontsize2)
#				figs[j][1].set_ylim((min(pos_data).value-1,max(pos_data).value+1))
				
				plt.draw()
				
				if j < config.n_joints - 1:
					figs[j][0].waitforbuttonpress(0.001)
			
				if quit_plot:
					break
			
			quit_plot = figs[-1][0].waitforbuttonpress()
			
		if quit_plot:
			break
	if quit_plot:
		break
			
print("Finish")

		
			
