#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
import config.config_create_trajectory as config
import config.config_plot_figures as config2
import numpy as np
import scipy.optimize as optimization
import math
import time as t

def fitting_func(t, a, A, gamma, omega, phi, v_min):
	result = []
	if not isinstance(t, (np.ndarray, np.generic)):
		if isinstance(t, list):
			t = np.array(t)
		else:
			t = np.array([t])
	
	for timestamp in t:
		result.append(v_min + a*timestamp + A*(math.exp(gamma * timestamp) * math.cos(omega*timestamp + phi) - math.cos(phi)))
	return np.array(result)

start_time = t.time()
demo_to_analyze = 0

## Settings
impact_detection_delay = 0.028
#impact_detection_delay = 0
impact_duration = 0.2
#impact_duration = 0
##

position_datasets = []
velocity_datasets = []

print("Reading demonstration data")

for j in [demo_to_analyze]:
	demo = config.demos[j]
	franka_reader = FrankaStateReader(demo)

	position_dataset = PositionDataSet()
	velocity_dataset = PositionDataSet()

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		position_dataset.append(PositionDataPoint(time, dp.position))
		velocity_dataset.append(PositionDataPoint(time, dp.velocity))
		
	position_dataset.align_time()
	velocity_dataset.align_time()
		
	position_datasets.append(position_dataset)
	velocity_datasets.append(velocity_dataset)
	
print("--- %s seconds ---" % (t.time() - start_time))
print("Analyzing data")

position_data = position_datasets[demo_to_analyze]
velocity_data = velocity_datasets[demo_to_analyze]

ending_index = config.impact_intervals[demo_to_analyze][0][0]
ending_impact_time = position_data[ending_index].time
while position_data[ending_index].time > ending_impact_time - impact_detection_delay:
	ending_index -= 1

starting_index = config.impact_intervals[demo_to_analyze][0][-1] + 1
starting_impact_time = position_data[starting_index-1].time
while position_data[starting_index-1].time > starting_impact_time - impact_detection_delay:
	starting_index -= 1
#starting_index = ending_index + 1

ante_impact_position = position_data[:ending_index].copy()
impact_phase_position = position_data[ending_index:starting_index].copy()
post_impact_position = position_data[starting_index:].copy()
detected_impact_position = PositionDataSet([position_data[i] for i in config.impact_intervals[demo_to_analyze][0]]).copy()
ante_impact_velocity = velocity_data[:ending_index].copy()
impact_phase_velocity = velocity_data[ending_index:starting_index].copy()
post_impact_velocity = velocity_data[starting_index:].copy()
detected_impact_velocity = PositionDataSet([velocity_data[i] for i in config.impact_intervals[demo_to_analyze][0]]).copy()

impact_phase_ending_index = 0
last_impact_time = impact_phase_position[-1].time
while post_impact_position[impact_phase_ending_index].time < last_impact_time + impact_duration:
	impact_phase_ending_index += 1
data_to_fit = post_impact_velocity[:impact_phase_ending_index].z
time_shift = data_to_fit[0].time
data_to_fit.align_time()

ante_impact_velocity_value = impact_phase_velocity[-1].z
func = lambda t, a, A, gamma, omega, phi : fitting_func(t, a, A, gamma, omega, phi, ante_impact_velocity_value)
p, pcov = optimization.curve_fit(func, np.array(data_to_fit.time), np.array(data_to_fit.value),maxfev=50000)

fitted_data = DataSet()
fitted_data_values = func(data_to_fit.time, p[0], p[1], p[2], p[3], p[4])
for i in range(len(data_to_fit.time)):
	fitted_data.append(DataPoint(data_to_fit.time[i] + time_shift, fitted_data_values[i]))

post_impact_prediction_data = DataSet()
post_impact_prediction_data_values = func(data_to_fit.time, p[0], p[1], 0, 0, p[4])
for i in range(len(data_to_fit.time)):
	post_impact_prediction_data.append(DataPoint(data_to_fit.time[i] + time_shift, post_impact_prediction_data_values[i] - p[1]*math.cos(p[4])))
	
post_impact_prediction_point_value = func(0, p[0], p[1], 0, 0, p[4]) - p[1]*math.cos(p[4])
post_impact_prediction_point = DataPoint(time_shift, post_impact_prediction_point_value)
	
data_to_fit.align_time(time_shift)

print("--- %s seconds ---" % (t.time() - start_time))
print("Plotting figures")
		
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
plt.rcParams['xtick.labelsize'] = config2.fontsize2
plt.rcParams['ytick.labelsize'] = config2.fontsize2
plt.plot(ante_impact_position.time, ante_impact_position.z.value, 'C1-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Ante-impact phase')
plt.plot(impact_phase_position.time, impact_phase_position.z.value, 'C2-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Impact phase')
plt.plot(detected_impact_position.time, detected_impact_position.z.value, 'C2*', markersize=config2.markersize1,label='Detected impacts')
plt.plot(post_impact_position.time, post_impact_position.z.value, 'C4-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact phase')
plt.legend(fontsize=config2.fontsize2)
plt.xlabel('Time [s]',fontsize=config2.fontsize2)
plt.ylabel('Position [m]',fontsize=config2.fontsize2)
plt.title('Z Position',fontsize=config2.fontsize1)
if config.xlim is not None:
	plt.xlim(config.xlim[demo_to_analyze])
	
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
plt.rcParams['xtick.labelsize'] = config2.fontsize2
plt.rcParams['ytick.labelsize'] = config2.fontsize2
plt.plot(ante_impact_velocity.time, ante_impact_velocity.z.value, 'C1-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Ante-impact phase')
plt.plot(impact_phase_velocity.time, impact_phase_velocity.z.value, 'C2-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Impact phase')
plt.plot(detected_impact_velocity.time, detected_impact_velocity.z.value, 'C2*', markersize=config2.markersize1,label='Detected impacts')
plt.plot(post_impact_velocity.time, post_impact_velocity.z.value, 'C4-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact phase')
plt.legend(fontsize=config2.fontsize2)
plt.xlabel('Time [s]',fontsize=config2.fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=config2.fontsize2)
plt.title('Z Velocity',fontsize=config2.fontsize1)
if config.xlim is not None:
	plt.xlim(config.xlim[demo_to_analyze])
	
plt.figure(figsize=config2.figsize,dpi=config2.dpi)
plt.rcParams['xtick.labelsize'] = config2.fontsize2
plt.rcParams['ytick.labelsize'] = config2.fontsize2
plt.plot(data_to_fit.time, data_to_fit.value, 'C4-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact phase')
plt.plot(fitted_data.time, fitted_data.value, 'C5-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Fitted data')
plt.plot(post_impact_prediction_data.time, post_impact_prediction_data.value, 'C6-*', linewidth=config2.linewidth, markersize=config2.markersize3,label='Post-impact velocity prediction fit')
plt.plot(post_impact_prediction_point.time, post_impact_prediction_point.value, 'C6*', markersize=config2.markersize1,label='Post-impact velocity prediction')
plt.legend(fontsize=config2.fontsize2)
plt.xlabel('Time [s]',fontsize=config2.fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=config2.fontsize2)
plt.title('Predicting the post-impact Velocity',fontsize=config2.fontsize1)
#if config2.xlim is not None:
#	plt.xlim(config2.xlim[demo_to_analyze])

print("--- %s seconds ---" % (t.time() - start_time))

plt.show()

print("Done")

