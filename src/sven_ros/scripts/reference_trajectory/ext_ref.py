#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from datalib import *
from filters import *
from trajectory import *

if __name__ == '__main__':
	data = DataSet([DataPoint(0,1), DataPoint(1,2), DataPoint(2,3), DataPoint(3,4), DataPoint(4,5), DataPoint(5,6), DataPoint(6,7), DataPoint(7,8), DataPoint(8,9), DataPoint(9,10)])
	
	estimator = LeastSquaresVelocityEstimator(window_length=5, order=3)
	extender = ConstantVelocityExtender(estimator, timesteps=3, delta_time=1)
	
	extended_data = extender.extend(data) + 0.1
	
	x0,y0 = data.get_xy()
	x1,y1 = extended_data.get_xy()
	plt.plot(x0,y0)
	plt.plot(x1,y1)
	plt.show()
	
