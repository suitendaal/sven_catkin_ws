#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from datalib.datapoint import DataPoint
from datalib.dataset import DataSet
from readers.joint_reader import JointReader

def main(bagfile):
	pos_data = []
	vel_data = []

	for i in range(7):
		reader = JointReader(bagfile,i+1)
		joint_data = reader.read()
		joint_pos_data = DataSet(timefactor=1000000)

		# Position data is index 0 of each datapoint
		for datapoint in joint_data:
			joint_pos_data.append(datapoint[0])
		
		joint_pos_data.align_time()
		pos_data.append(joint_pos_data)
		
		joint_vel_data = DataSet(timefactor=1000000)
		for datapoint in joint_data:
			joint_vel_data.append(datapoint[1])
		
		joint_vel_data.align_time()
		vel_data.append(joint_vel_data)
	
	for i in range(7):
		plt.figure(2*i)
		x, y = (pos_data[i] - pos_data[i][0]).get_xy()
		plt.plot(x, y)
		plt.figure(2*i+1)
		x, y = vel_data[i].get_xy()
		plt.plot(x, y)
	
	plt.show()

	return pos_data, vel_data

if __name__ == '__main__':
	if len(sys.argv) == 2:
		pos_data, vel_data = main(sys.argv[1])

