#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from readers import *
from datalib import *
from filters import *

def get_data(bagfile, joint):
	reader = JointReader(bagfile,joint)
	joint_data = reader.read()
	joint_pos_data = DataSet(timefactor=1000000)
	joint_vel_data = DataSet(timefactor=1000000)
	joint_eff_data = DataSet(timefactor=1000000)

	# Position data is index 0 of each datapoint
	for datapoint in joint_data:
		joint_pos_data.append(datapoint[0])
		joint_vel_data.append(datapoint[1])
		joint_eff_data.append(datapoint[2])
		
	joint_pos_data.align_time()
	joint_vel_data.align_time()
	joint_eff_data.align_time()
	return joint_pos_data, joint_vel_data, joint_eff_data
	
if __name__ == '__main__':
	if len(sys.argv) != 3:
		print("Usage: ja_filter.py <bagfile> <joint>")
		exit(1)
	bagfile = sys.argv[1]
	joint = int(sys.argv[2])
	pos_data, vel_data, eff_data = get_data(bagfile,joint)
	
	filter = LeastSquaresFilter(window_length=10, order=3)
	vel_estimator = LeastSquaresVelocityEstimator(window_length=10, order=3)
	predictor = Predictor(order=3)
	bounder = Bounder(bound=0.005)
	jafilter = JumpAwareFilter(filter, vel_estimator, predictor, bounder, max_window_length=20, time_step=0.01)

	filtered_data, vel_estimation, jumping_indexes, info = jafilter.filter(pos_data)
	predictions = info[0]
	bounds = info[1]
	
	starting_time = 0
	if len(jumping_indexes) > 0:
		starting_time = -filtered_data[jumping_indexes[0]].time
	
	plt.figure(1)
	
	x0,y0 = (pos_data - filtered_data[0]).get_xy()
	plt.plot(x0,y0)
	
	x1,y1 = (filtered_data - filtered_data[0]).get_xy()
	plt.plot(x1,y1)
	
	pos_jump = DataSet([filtered_data[index] for index in jumping_indexes],timefactor=1000000)
	pos_jump.align_time(starting_time)
	x2,y2 = (pos_jump - filtered_data[0]).get_xy()
	plt.plot(x2,y2,'*')
	
	x11,y11 = (predictions - filtered_data[0]).get_xy()
	plt.plot(x11,y11)
	
	plt.figure(2)
	
	x3,y3 = vel_data.get_xy()
	plt.plot(x3,y3)
	
	x3_1,y3_1 = vel_estimation.get_xy()
	plt.plot(x3_1,y3_1)
	
	vel_jump = DataSet([vel_data[index] for index in jumping_indexes],timefactor=1000000)
	vel_jump.align_time(starting_time)
	x4,y4 = vel_jump.get_xy()
	plt.plot(x4,y4,'*')
	
	plt.figure(3)
	
	x5,y5 = eff_data.get_xy()
	plt.plot(x5,y5)
	
	eff_jump = DataSet([eff_data[index] for index in jumping_indexes],timefactor=1000000)
	eff_jump.align_time(starting_time)
	x6,y6 = eff_jump.get_xy()
	plt.plot(x6,y6,'*')
	
	plt.show()
	
