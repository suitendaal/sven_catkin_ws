#!/usr/bin/python3

from . import *

def get_cartesian_data(bagfile):
	reader = CartesianPoseReader(bagfile)
	data = reader.read()
	x_data = DataSet(timefactor=1000000)
	y_data = DataSet(timefactor=1000000)
	z_data = DataSet(timefactor=1000000)
	a_data = DataSet(timefactor=1000000)
	b_data = DataSet(timefactor=1000000)
	c_data = DataSet(timefactor=1000000)
	d_data = DataSet(timefactor=1000000)

	# Position data is index 0 of each datapoint
	for datapoint in data:
		x_data.append(datapoint[0])
		y_data.append(datapoint[1])
		z_data.append(datapoint[2])
		a_data.append(datapoint[3])
		b_data.append(datapoint[4])
		c_data.append(datapoint[5])
		d_data.append(datapoint[6])
		
	x_data.align_time()
	y_data.align_time()
	z_data.align_time()
	a_data.align_time()
	b_data.align_time()
	c_data.align_time()
	d_data.align_time()
	return x_data, y_data, z_data, [a_data, b_data, c_data, d_data]
	
def get_joint_data(bagfile, joint):
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
