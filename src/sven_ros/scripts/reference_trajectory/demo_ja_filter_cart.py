#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from readers import *
from datalib import *
from filters import *	
	
if __name__ == '__main__':
	if len(sys.argv) != 2:
		print("Usage: ja_filter.py <bagfile>")
		exit(1)
	bagfile = sys.argv[1]
	x_data, y_data, pos_data, quat_data = get_cartesian_data(bagfile)
	
	filter = LeastSquaresFilter(window_length=20, order=3)
	vel_estimator = LeastSquaresVelocityEstimator(window_length=20, order=3)
	predictor = Predictor(order=3)
	bounder = Bounder(bound=0.0015)
	jafilter = JumpAwareFilter(filter, vel_estimator, predictor, bounder, max_window_length=20, time_step=0.01)

	filtered_data, vel_estimation, jumping_indexes, info = jafilter.filter(pos_data)
	predictions = info[0]
	bounds = info[1]
	prediction_functions = info[2]
	bound_functions = info[3]
	
	predictions2 = DataSet()
	for i in prediction_functions:
		predictions2.append(i.evaluate()[0])
		
	bounds2 = DataSet()
	for i in bound_functions:
		bounds2.append(i.evaluate()[0])
	
	starting_time = 0
	starting_time1 = 0
	if len(jumping_indexes) > 0:
		starting_time = -filtered_data[jumping_indexes[0]].time
		starting_time1 = -filtered_data[jumping_indexes[0]-1].time
		
	xlim = [pos_data[0].time, pos_data[-1].time]
	ylim = [-0.145, -0.125]
	fontsize1 = 20
	fontsize2 = 16
	
	## Plot position data, filtered data and predictions
	fig1 = plt.figure(1,figsize=(16, 12), dpi=80)
	plt.rcParams['xtick.labelsize']=fontsize2
	plt.rcParams['ytick.labelsize']=fontsize2
	
	# Position data
	x0,y0 = (pos_data - filtered_data[0]).get_xy()
	plt.plot(x0,y0,'C0-',linewidth=2)
	
	# Filtered data
	x1,y1 = (filtered_data - filtered_data[0]).get_xy()
	plt.plot(x1,y1,'C1-',linewidth=2)
	
	# Predicted data
	x11,y11 = (predictions - filtered_data[0]).get_xy()
	plt.plot(x11,y11,'C2-*',linewidth=2)
	
	# Jump times
	pos_jump = DataSet([filtered_data[index].copy() for index in jumping_indexes],timefactor=1000000).align_time(starting_time)
	x2,y2 = (pos_jump - filtered_data[0]).get_xy()
	plt.plot(x2,y2,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Z direction: Position',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Position [m]',fontsize=fontsize2)
	plt.legend(['Encoder','Filtered','Predicted','Jumps'],fontsize=fontsize2)
	plt.xlim(xlim)
	#plt.ylim(ylim)
	
	## Plot difference between prediction and encoder data
	plt.figure(2,figsize=(16, 12), dpi=80)
	plt.rcParams['xtick.labelsize']=fontsize2
	plt.rcParams['ytick.labelsize']=fontsize2
	
	# Difference between prediction and encoder data
	x3,y3 = (abs(pos_data - predictions)).get_xy()
	plt.plot(x3,y3,'C0-',linewidth=2)
	
	# Bound
	x4,y4 = bounds.get_xy()
	plt.plot(x4,y4,'C1-',linewidth=2)
	
	# Jump times
	x5,y5 = DataSet([abs(pos_data - predictions)[index] for index in jumping_indexes],timefactor=1000000).align_time(starting_time).get_xy()
	plt.plot(x5,y5,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Z direction: Absolute difference between measurement and prediction',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Difference [m]',fontsize=fontsize2)
	plt.legend(['Difference','Bound','Jumps'],fontsize=fontsize2)
	plt.xlim(xlim)
	
	## Plot velocity data
	plt.figure(3,figsize=(16, 12), dpi=80)
	plt.rcParams['xtick.labelsize']=fontsize2
	plt.rcParams['ytick.labelsize']=fontsize2
	
#	# Velocity data
#	x6,y6 = vel_data.get_xy()
#	plt.plot(x6,y6,'C0-',linewidth=2)
	x6,y6 = pos_data.diff().get_xy()
	plt.plot(x6,y6)
	print(x6[0])
	
	# Estimated velocity
	x7,y7 = vel_estimation.get_xy()
	plt.plot(x7,y7,'C1-',linewidth=2)
	
	# Jumping times
	x8,y8 = DataSet([vel_estimation[index-1].copy() for index in jumping_indexes],timefactor=1000000).align_time(starting_time1).get_xy()
	plt.plot(x8,y8,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Z direction: Velocity',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
	plt.legend(['Euler differentiation','Estimation','Jumps'],fontsize=fontsize2)
	plt.xlim(xlim)
	
	plt.figure(4)

	x9,y9 = (predictions2 - filtered_data[0]).get_xy()
	x10,y10 = bounds2.get_xy()
#	plt.plot(x10,y10)
	plt.plot(x9,y9)
	plt.plot(x11,y11)
#	
#	eff_jump = DataSet([eff_data[index] for index in jumping_indexes],timefactor=1000000)
#	eff_jump.align_time(starting_time)
#	x6,y6 = eff_jump.get_xy()
#	plt.plot(x6,y6,'*')
	
	plt.show()
	
