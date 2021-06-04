#!/usr/bin/python3

import sys
import matplotlib.pyplot as plt
from datalib.dataset import DataSet
from datalib.datapoint import DataPoint
from filters.filters.least_squares_filter import *
from filters.velocity_estimator.least_squares_velocity_estimator import *

if __name__ == '__main__':
	data = DataSet([DataPoint(0,1), DataPoint(1,2), DataPoint(2,3), DataPoint(3,4), DataPoint(4,5), DataPoint(5,1), DataPoint(6,2), DataPoint(7,3), DataPoint(8,4), DataPoint(9,5)])
	
	filter_config = LeastSquaresFilterConfiguration(window_length = 5, order = 3)
	filter = LeastSquaresFilter(filter_config)
	filtered_data = filter.filter(data)
	plt.figure(1)
	x0,y0 = data.get_xy()
	plt.plot(x0,y0)
	x1,y1 = filtered_data.get_xy()
	plt.plot(x1,y1)
	
	vel_config = LeastSquaresVelocityEstimatorConfiguration(window_length=5, order=3)
	estimator = LeastSquaresVelocityEstimator(vel_config)
	vel_data = estimator.estimate(data)
	plt.figure(2)
	x2,y2 = vel_data.get_xy()
	plt.plot(x2,y2)
	plt.show()
