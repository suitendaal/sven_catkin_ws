#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from readers import *
from datalib import *
from filters import *
	
if __name__ == '__main__':
	if len(sys.argv) != 3:
		print("Usage: ja_filter.py <bagfile> <joint>")
		exit(1)
	bagfile = sys.argv[1]
	joint = int(sys.argv[2])
	pos_data, vel_data, eff_data = get_joint_data(bagfile,joint)
	
	print(pos_data)
	
	filter = LeastSquaresFilter(window_length=10, order=3)
	vel_estimator = LeastSquaresVelocityEstimator(window_length=10, order=3)
	predictor = BasePredictor(order=3)
	bounder = BaseBounder(bound=0.005)
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
	
