#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *
import config.config_jump_detection as config

bagfile = 'data/replay4.1.bag'
franka_reader = FrankaStateReader(bagfile)

rotation = []
drotation = []
for i in range(3):
	rotation.append(DataSet())
	drotation.append(DataSet())

for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	time = dp.time
	dp = dp.value
	
	for j in range(3):
		rotation[j].append(DataPoint(time, dp.euler_angles[j]))
		drotation[j].append(DataPoint(time, dp.rotational_velocity[j]))
		
for i in range(3):
	rotation[i].align_time()
	drotation[i].align_time()
	
for i in range(3):	
	plt.figure()
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	plt.plot(rotation[i].diff().time, rotation[i].diff().value,'C0-*',linewidth=config.linewidth, markersize=config.markersize2, label="Euler")
	plt.plot(drotation[i].time, drotation[i].value,'C1-*',linewidth=config.linewidth, markersize=config.markersize2, label="Data")
	plt.legend(fontsize=config.fontsize2)
	if config.xlim[0] is not None:
		plt.xlim(config.xlim[0])
	plt.title('Velocity ' + config.labels[i],fontsize=config.fontsize1)
	
plt.show()

