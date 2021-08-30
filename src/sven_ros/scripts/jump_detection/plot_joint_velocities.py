#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *
import config.config_jump_detection as config

bagfile = 'data/replay4.1.bag'
franka_reader = FrankaStateReader(bagfile)

	
qs = []
dqs = []
dqs_filtered = []
dqs_est = []
for i in range(7):
	qs.append(DataSet())
	dqs.append(DataSet())
	dqs_filtered.append(DataSet())
	dqs_est.append(DataSet())

for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	time = dp.time
	dp = dp.value
	
	for j in range(7):
		qs[j].append(DataPoint(time, dp.q[j]))
		dqs[j].append(DataPoint(time, dp.dq[j]))
		
for i in range(7):
	qs[i].align_time()
	dqs[i].align_time()
	dqs_filtered[i].align_time()
	dqs_est[i].align_time()
		
vel_estimator = LeastSquaresVelocityEstimator(window_length=20, order=2)
for i in range(7):
	vel_estimator.reset()
	for j in range(len(qs[i])):
		vel_est, coefs = vel_estimator.update(qs[i][j])
		dqs_est[i].append(vel_est)
		
vel_filter = LeastSquaresFilter(window_length=20, order=2)
for i in range(7):
	vel_filter.reset()
	for j in range(len(qs[i])):
		vel, coefs = vel_filter.update(dqs[i][j])
		dqs_filtered[i].append(vel)
	
for i in range(7):	
	plt.figure()
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	plt.plot(dqs_est[i].time, dqs_est[i].value,'C0-*',linewidth=config.linewidth, markersize=config.markersize2, label="Estimation")
	plt.plot(dqs_filtered[i].time, dqs_filtered[i].value,'C1-*',linewidth=config.linewidth, markersize=config.markersize2, label="Filtered")
	plt.plot(dqs[i].time, dqs[i].value,'C2-*',linewidth=config.linewidth, markersize=config.markersize2, label="Data")
	plt.plot(qs[i].diff().time, qs[i].diff().value,'C3-*',linewidth=config.linewidth, markersize=config.markersize2, label="Euler")
	plt.legend(fontsize=config.fontsize2)
	if config.xlim[0] is not None:
		plt.xlim(config.xlim[0])
	plt.title('Joint ' + str(i+1) + ' Velocity',fontsize=config.fontsize1)
	
plt.show()

