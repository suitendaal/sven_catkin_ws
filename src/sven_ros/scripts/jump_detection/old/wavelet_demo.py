#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
import pywt
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
	
	t = pos_data.time()
	sig = pos_data.values()
	widths = np.arange(1, 6)
	cwtmatr, freqs = pywt.cwt(sig, widths, 'gaus1', sampling_period=0.01, method='conv')
	cwtmatr *= 1 / (widths[:,None] ** 0.5)
	
	plt.figure(1,figsize=(16, 12), dpi=80)
	plt.imshow(cwtmatr, extent=[t[0], t[-1], widths[-1], widths[0]], cmap='PRGn', aspect='auto', vmax=abs(cwtmatr[:60,100:700]).max(), vmin=-abs(cwtmatr[:60,100:700]).max())
	
	plt.figure(2,figsize=(16, 12), dpi=80)
	plt.plot(t,cwtmatr[0,:] * -1000 / freqs[0])
	plt.plot(vel_data.time(),vel_data.values())
	plt.ylim([-5,5])
	plt.legend(['Wavelet','Velocity'])
	
	print(freqs)
	
	plt.show()

