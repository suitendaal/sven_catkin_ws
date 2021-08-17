#!/usr/bin/python3

from . import *

def get_cartesian_data(bagfile):
	reader = CartesianPoseReader(bagfile)
	data = reader.read()
	
	# [x,y,z,q1,q2,q3,q4]
	datasets = get_datasets(data,7)
		
	# [x,y,z,q]
	return datasets[0], datasets[1], datasets[2], datasets[3:7]
	
def get_joint_data(bagfile, joint):
	reader = JointReader(bagfile,joint)
	data = reader.read()
	
	# [x,v,tau]
	datasets = get_datasets(data,3)
	
	return datasets[0], datasets[1], datasets[2]
	
def get_datasets(data, num):
	datasets = []
	time_compensations = []
	
	for i in range(num):
		datasets.append(DataSet())
		time_compensations.append(0)

	for i in range(len(data)):
		for j in range(len(datasets)):
			if i == 0 or data[i][j].value != data[i-1][j].value:
				datapoint = data[i][j].copy()
				datapoint.timestamp = data[i].timestamp - time_compensations[j]
				datasets[j].append(datapoint)
			else:
				time_compensations[j] = time_compensations[j] + data[i].timestamp - data[i-1].timestamp
		
	for i in range(len(datasets)):
		datasets[i].align_time()
	
	return datasets
