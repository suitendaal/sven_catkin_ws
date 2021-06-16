#!/usr/bin/python3

import functools
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
		
	def copy(self):
		return JumpAwareFilter(self._filter.copy(), self.velocity_estimator.copy(), self.predictor.copy(), self.bounder.copy(), max_window_length=self.max_window_length)

	# Filter the data and predict jumping time indexes
	def filter(self, data):
		filtered_data = DataSet(timefactor=data.timefactor)
		vel_data = DataSet(timefactor=data.timefactor)
		jumping_indexes = []
		
		predictions = DataSet(timefactor=data.timefactor)
		bounds = DataSet(timefactor=data.timefactor)
		
		prediction_functions = DataSet(timefactor=data.timefactor)
		bound_functions = DataSet(timefactor=data.timefactor)

		window_length = 0
		prediction = None
		prediction_fun = None
		bound = None
		bound_fun = None

		for i in range(len(data)):
			if i > 0:
				time = data[i].time
				if prediction_fun is not None:
					prediction, pred_info = prediction_fun(time)
				else:
					prediction = None
				if bound_fun is not None:
					bound, bound_info = bound_fun(time)
				else:
					bound = None
			
			if prediction_fun is not None:
				prediction_functions.append(FunctionPoint(data[i].timestamp, prediction_fun), reset_time=False)
			else:
				prediction_functions.append(FunctionPoint(data[i].timestamp, None), reset_time=False)
			if bound_fun is not None:
				bound_functions.append(FunctionPoint(data[i].timestamp, bound_fun), reset_time=False)
			else:
				bound_functions.append(FunctionPoint(data[i].timestamp, None), reset_time=False)
		
			predictions.append(DataPoint(data[i].timestamp, prediction), reset_time=False)
			bounds.append(DataPoint(data[i].timestamp, bound), reset_time=False)
		
			# Check if jump has occured
			if prediction is not None and bound is not None and abs(data[i].value - prediction) >= bound:
				window_length = 0
				jumping_indexes.append(i)

			# Filter the data
			start = max(0, i - window_length)
			end = i + 1
			dataset = data[start:end].copy()
			filtered_datapoint = self._filter.filter(dataset, window_length)[0][-1]
			filtered_data.append(filtered_datapoint, reset_time=False)
			vel_estimation = self.velocity_estimator.estimate(dataset, window_length)[0][-1]
			vel_data.append(vel_estimation, reset_time=False)

			# Predict next datapoint
			if window_length > 1:
				prediction_fun = functools.partial(self.predictor.predict, dataset.copy(), window_length)
				bound_fun = functools.partial(self.bounder.calc_bound, dataset.copy(), window_length)
			else:
				prediction_fun = None
				bound_fun = None

			window_length = min(window_length + 1, self.max_window_length)
		
		
		filtered_data.align_time()
		vel_data.align_time()
		predictions.align_time()
		bounds.align_time()
		prediction_functions.align_time()
		bound_functions.align_time()

		return filtered_data, vel_data, jumping_indexes, [predictions, bounds, prediction_functions, bound_functions]

