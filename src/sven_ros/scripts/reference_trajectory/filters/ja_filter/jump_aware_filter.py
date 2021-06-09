#!/usr/bin/python3

from datalib import *
from filters import *

class JumpAwareFilter(object):
	"""docstring for JumpAwareFilter."""

	def __init__(self, filter, velocity_estimator, predictor, bounder, **kwargs):
		super(JumpAwareFilter, self).__init__()
		self._filter = filter
		self.velocity_estimator = velocity_estimator
		self.predictor = predictor
		self.bounder = bounder
		self.max_window_length = kwargs.get('max_window_length',10)
		self.time_step = kwargs.get('time_step',0.01)

	# Filter the data and predict jumping time indexes
	def filter(self, data):
		filtered_data = DataSet(timefactor=data.timefactor)
		vel_data = DataSet(timefactor=data.timefactor)
		jumping_indexes = []
		
		predictions = DataSet(timefactor=data.timefactor)
		bounds = DataSet(timefactor=data.timefactor)

		window_length = 0
		prediction = None
		prediction_fun = None
		bound = None
		bound_fun = None

		for i in range(len(data)):
			if i > 0:
				time_step = data[i].time - data[i-1].time
				if prediction_fun is not None:
					prediction = prediction_fun(time_step)
				else:
					prediction = None
				if bound_fun is not None:
					bound = bound_fun(time_step)
				else:
					bound = None
		
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
				prediction_fun = lambda time_step : self.predictor.predict(dataset, window_length, time_step)
				bound_fun = lambda time_step : self.bounder.calc_bound(dataset, window_length, time_step)
			else:
				prediction_fun = None
				bound_fun = None

			window_length = min(window_length + 1, self.max_window_length)
		
		
		filtered_data.align_time()
		vel_data.align_time()
		predictions.align_time()
		bounds.align_time()

		return filtered_data, vel_data, jumping_indexes, [predictions, bounds]

