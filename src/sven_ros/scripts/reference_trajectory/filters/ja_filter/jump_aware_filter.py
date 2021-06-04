#!/usr/bin/python3

from datalib import *
from filters import *

class JumpAwareFilter(object):
	"""docstring for JumpAwareFilter."""

	def __init__(self, filter, velocity_estimator, predictor, bounder, config):
		super(JumpAwareFilter, self).__init__()
		self._filter = filter
		self.velocity_estimator = velocity_estimator
		self.predictor = predictor
		self.bounder = bounder
		self.config = config

	# Filter the data and predict jumping time indexes
	def filter(self, data):
		filtered_data = DataSet(timefactor=data.timefactor)
		vel_data = DataSet(timefactor=data.timefactor)
		jumping_indexes = []
		
		predictions = DataSet(timefactor=data.timefactor)
		bounds = DataSet(timefactor=data.timefactor)

		window_length = 0
		prediction = None
		bound = None

		for i in range(len(data)):
			predictions.append(DataPoint(data[i].timestamp, prediction))
			bounds.append(DataPoint(data[i].timestamp, bound))
		
			# Check if jump has occured
			if prediction is not None and bound is not None and abs(data[i].value - prediction) >= bound:
				window_length = 0
				jumping_indexes.append(i)

			# Filter the data
			start = max(0, i - window_length)
			end = i + 1
			dataset = data[start:end]
			filtered_datapoint = self._filter.filter(dataset, window_length)[-1]
			filtered_data.append(filtered_datapoint)
			vel_estimation = self.velocity_estimator.estimate(dataset, window_length)[-1]
			vel_data.append(vel_estimation)

			# Predict next datapoint
			if window_length > 1:
				prediction = self.predictor.predict(dataset, window_length, self.config.time_step)
				bound = self.bounder.calc_bound(dataset, window_length, self.config.time_step)
			else:
				prediction = None
				bound = None

			window_length = min(window_length + 1, self.config.max_window_length)
		
		
		filtered_data.align_time()
		vel_data.align_time()
		predictions.align_time()
		bounds.align_time()

		return filtered_data, vel_data, jumping_indexes, [predictions, bounds]

class JumpAwareFilterConfiguration(object):
	"""docstring for JumpAwareFilterConfiguration."""

	def __init__(self, max_window_length=10, time_step=0.001):
		super(JumpAwareFilterConfiguration, self).__init__()
		self.max_window_length = max_window_length
		self.time_step = time_step

def main():
	data = [DataPoint(0,1), DataPoint(1,2), DataPoint(2,3), DataPoint(3,4), DataPoint(4,5), DataPoint(5,1), DataPoint(6,2), DataPoint(7,3), DataPoint(8,4), DataPoint(9,5)]

	filter_config = LeastSquaresFilterConfiguration(window_length = 5, order = 3)
	filter = LeastSquaresFilter(filter_config)
	
	vel_config = LeastSquaresVelocityEstimatorConfiguration(window_length = 5, order = 3)
	vel_estimator = LeastSquaresVelocityEstimator(vel_config)

	predictor_config = PredictorConfiguration(order = 3)
	predictor = Predictor(predictor_config)

	bounder_config = BounderConfiguration(bound = 0.3)
	bounder = Bounder(bounder_config)

	config = JumpAwareFilterConfiguration(max_window_length=10, time_step=1)
	jafilter = JumpAwareFilter(filter, vel_estimator, predictor, bounder, config)

	filtered_data, vel_estimation, jumping_times, info = jafilter.filter(data)

	print(filtered_data)
	print(jumping_times)

if __name__ == '__main__':
	main()
