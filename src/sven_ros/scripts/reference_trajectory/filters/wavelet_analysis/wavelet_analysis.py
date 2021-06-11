#!/usr/bin/python3

import numpy as np
#import pywt
from datalib import *
#from trajectory import RadialBasisFunction as RBF

def wavelet(points, a):
	order = 2
	width = a**2
	vec = np.arange(0, points) - (points - 1.0) / 2
	
#	A = 2 / (np.sqrt(3 * a) * (np.pi**0.25))
#	xsq = vec**2
#	mod = (1 - xsq / wsq)
#	gauss = np.exp(-xsq / (2 * wsq))
#	total = A * mod * np.array(gauss)
	
	A = 1 / (a * np.sqrt(2 * np.pi)) * ((-width) ** order)
	rbf = RBF(width=width)
	gauss = []
	for i in vec:
		gauss.append(-rbf.evaluate(i,derivative=order))
	total = A * np.array(gauss)
	return total

