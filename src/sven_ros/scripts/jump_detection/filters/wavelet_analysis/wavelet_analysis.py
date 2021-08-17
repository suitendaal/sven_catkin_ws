#!/usr/bin/python3

import numpy as np
from datalib import *
from trajectory import RadialBasisFunction as RBF

def wavelet(points, a):
	
	# 3th order derivative (jerk)
	order = 3
	
	# Width and datapoints of the Gaussian
	width = a**2
	vec = np.arange(0, points) - (points - 1.0) / 2

	# Normalization factor	
	A = 1
	
	# Gaussian function
	rbf = RBF(width=width)
	gauss = []
	for i in vec:
		gauss.append(rbf.evaluate(i,derivative=order))
		
	return A * np.array(gauss)

