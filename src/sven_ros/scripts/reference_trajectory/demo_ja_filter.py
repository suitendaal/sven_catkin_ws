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
	
	filter = LeastSquaresFilter(window_length=20, order=3)
	vel_estimator = LeastSquaresVelocityEstimator(window_length=20, order=3)
	predictor = Predictor(order=3)
	bounder = Bounder(bound=0.005)
	jafilter = JumpAwareFilter(filter, vel_estimator, predictor, bounder, max_window_length=20, time_step=0.01)

	filtered_data, vel_estimation, jumping_indexes, info = jafilter.filter(pos_data)
	predictions = info[0]
	bounds = info[1]
	
	starting_time = 0
	if len(jumping_indexes) > 0:
		starting_time = -filtered_data[jumping_indexes[0]].time
		
	xlim = [4.9, 5.7]
	
	## Plot position data, filtered data and predictions
	fig1 = plt.figure(1)
	
	# Position data
	x0,y0 = (pos_data - filtered_data[0]).get_xy()
	plt.plot(x0,y0,'C0-',linewidth=2)
	
	# Filtered data
	x1,y1 = (filtered_data - filtered_data[0]).get_xy()
	plt.plot(x1,y1,'C1-',linewidth=2)
	
	# Predicted data
	x11,y11 = (predictions - filtered_data[0]).get_xy()
	plt.plot(x11,y11,'C2-',linewidth=2)
	
	# Jump times
	pos_jump = DataSet([filtered_data[index] for index in jumping_indexes],timefactor=1000000).align_time(starting_time)
	x2,y2 = (pos_jump - filtered_data[0]).get_xy()
	plt.plot(x2,y2,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Joint ' + str(joint) + ': Position')
	plt.xlabel('Time [s]')
	plt.ylabel('Position [rad]')
	plt.legend(['Encoder','Filtered','Predicted','Jumps'])
	plt.xlim(xlim)
	
	## Plot difference between prediction and encoder data
	plt.figure(2)
	
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
	plt.title('Joint ' + str(joint) + ': Absolute difference between measurement and prediction')
	plt.xlabel('Time [s]')
	plt.ylabel('Difference [rad]')
	plt.legend(['Difference','Bound','Jumps'])
	plt.xlim(xlim)
	
	## Plot velocity data
	plt.figure(3)
	
	# Velocity data
	x6,y6 = vel_data.get_xy()
	plt.plot(x6,y6,'C0-',linewidth=2)
	
	# Estimated velocity
	x7,y7 = vel_estimation.get_xy()
	plt.plot(x7,y7,'C1-',linewidth=2)
	
	# Jumping times
	x8,y8 = DataSet([vel_data[index] for index in jumping_indexes],timefactor=1000000).align_time(starting_time).get_xy()
	plt.plot(x8,y8,'C3*',markersize=10)
	
	# Adding title and labels
	plt.title('Joint ' + str(joint) + ': Velocity')
	plt.xlabel('Time [s]')
	plt.ylabel('Velocity [rad/s]')
	plt.legend(['Encoder','Estimation','Jumps'])
	plt.xlim(xlim)
	
#	plt.figure(3)
#	
#	x5,y5 = eff_data.get_xy()
#	plt.plot(x5,y5)
#	
#	eff_jump = DataSet([eff_data[index] for index in jumping_indexes],timefactor=1000000)
#	eff_jump.align_time(starting_time)
#	x6,y6 = eff_jump.get_xy()
#	plt.plot(x6,y6,'*')
	
	plt.show()
	
