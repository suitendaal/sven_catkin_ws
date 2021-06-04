#!/usr/bin/python3

import numpy as np
from .velocity_estimator import *
from datalib import *

class LeastSquaresVelocityEstimator(VelocityEstimator):
	"""docstring for Filter."""

	def estimate(self, data, window_length=None):
		if window_length is None:
			window_length = self.config.window_length

		result = DataSet()
		for i in range(len(data)):
			start = max(0, i - window_length)
			end = i + 1
			order = min(end - start - 1, self.config.order)
			if order == 0:
				result.append(DataPoint(data[i].timestamp, None))
				continue
			subset = data[start:end]
			x = []
			y = []
			for j in subset:
				x.append(j.time)
				y.append(j.value)
			coefs = np.polyfit(x, y, order)
			value = 0
			time = data[i].time
			for j in range(len(coefs) - 1):
				power = len(coefs) - 1 - j
				coef = coefs[j]
				value = value + coef * power * (time ** (power - 1))
			result.append(DataPoint(data[i].timestamp, value))
		return result


class LeastSquaresVelocityEstimatorConfiguration(VelocityEstimatorConfiguration):
	"""docstring for FilterConfiguration."""

	def __init__(self, window_length=10, order=3):
		super().__init__(window_length)
		self.order = order

def main():
	data = [DataPoint(0,1), DataPoint(1,2), DataPoint(2,3), DataPoint(3,4), DataPoint(4,5), DataPoint(5,1), DataPoint(6,2), DataPoint(7,3), DataPoint(8,4), DataPoint(9,5)]
	config = LeastSquaresVelocityEstimatorConfiguration(window_length=5, order=3)
	estimator = LeastSquaresVelocityEstimator(config)
	print(estimator.estimate(data))

if __name__ == '__main__':
	main()
