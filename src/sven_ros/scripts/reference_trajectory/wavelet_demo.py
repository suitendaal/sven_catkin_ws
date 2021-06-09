#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from scipy import signal
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
	
	t = pos_data.time()
	sig = pos_data.values()
	widths = np.arange(1, 31)
	cwtmatr = signal.cwt(sig, signal.ricker, widths)
	
	print(cwtmatr.shape)

	plt.imshow(cwtmatr, extent=[t[0], t[-1], 1, 31], cmap='PRGn', aspect='auto',vmax=abs(cwtmatr).max(), vmin=-abs(cwtmatr).max())
	plt.show()
