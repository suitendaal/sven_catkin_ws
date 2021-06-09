#!/usr/bin/python3

import sys
import numpy as np
import matplotlib.pyplot as plt
from readers import *
from datalib import *
from trajectory import *

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
		
	rbfs = []
	nbvars = 160
	for i in range(nbvars):
		center = (pos_data[-1].time - pos_data[0].time) * i / (nbvars-1) + pos_data[0].time
		width = (pos_data[-1].time - pos_data[0].time) / (nbvars-1) / 2
		rbf = RadialBasisFunction(center=center , width=width)
		rbfs.append(rbf)
		
	dataset = DataSet()
	for i in range(len(pos_data)):
		datapoint = pos_data[i].copy()
		datapoint.value = [datapoint.value, vel_data[i].value]
		dataset.append(datapoint)
	
	promp = ProMP(rbfs,derivatives=1,weights_covariance=1)
	mu_w,sigma_w,mu = promp.learn([dataset])
	
	xlim = [4.9, 5.7]
	fontsize1 = 20
	fontsize2 = 16
	
	## Plot position data
	plt.figure(1)
	mu,sigma = promp.evaluate(pos_data.time(),via_points=DataSet([ViaPoint(2,2.3,derivative=0),ViaPoint(3,0.5,derivative=1)]))
	plt.plot(pos_data.time(),pos_data.values())
	plt.plot(pos_data.time(),mu)
	
	# Adding title and labels
	plt.title('Joint ' + str(joint) + ': Position',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Position [rad]',fontsize=fontsize2)
	plt.legend(['Encoder','ProMP'],fontsize=fontsize2)
	
	## Plot velocity data
	plt.figure(2)
	mu2,sigma2 = promp.evaluate(vel_data.time(),derivative=1,via_points=DataSet([ViaPoint(2,2.3,derivative=0),ViaPoint(3,0.5,derivative=1)]))
	plt.plot(vel_data.time(),vel_data.values())
	plt.plot(vel_data.time(),mu2)
	
	# Adding title and labels
	plt.title('Joint ' + str(joint) + ': Velocity',fontsize=fontsize1)
	plt.xlabel('Time [s]',fontsize=fontsize2)
	plt.ylabel('Velocity [rad/s]',fontsize=fontsize2)
	plt.legend(['Encoder','ProMP'],fontsize=fontsize2)
	
	plt.show()
	
	
