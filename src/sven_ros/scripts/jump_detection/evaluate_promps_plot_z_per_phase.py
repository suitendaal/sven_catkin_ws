#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_evaluate_promps as config
import config.config_create_trajectory as config2
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP
import time as t

start_time = t.time()

position_datasets = []
orientation_datasets = []
velocity_datasets = []
rotational_velocity_datasets = []

print("Reading demonstration data")

for demo in config2.demos:
	franka_reader = FrankaStateReader(demo)

	position_dataset = PositionDataSet()
	orientation_dataset = PositionDataSet()
	velocity_dataset = PositionDataSet()
	rotational_velocity_dataset = PositionDataSet()

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		position_dataset.append(PositionDataPoint(time, dp.position))
		orientation_dataset.append(PositionDataPoint(time, dp.euler_angles))
		velocity_dataset.append(PositionDataPoint(time, dp.velocity))
		rotational_velocity_dataset.append(PositionDataPoint(time, dp.rotational_velocity))
		
	position_dataset.align_time()
	orientation_dataset.align_time()
	velocity_dataset.align_time()
	rotational_velocity_dataset.align_time()
		
	position_datasets.append(position_dataset)
	orientation_datasets.append(orientation_dataset)
	velocity_datasets.append(velocity_dataset)
	rotational_velocity_datasets.append(rotational_velocity_dataset)
	
datasets_handle = RobotDataSets(position_datasets, velocity_datasets, orientation_datasets, rotational_velocity_datasets, config2.impact_intervals, config2.impact_detection_delay, config2.impact_phase_duration)

print("--- %s seconds ---" % (t.time() - start_time))
print("Filtering and extending demonstration data")

datasets_handle.filter_position_data(config2.position_filter)
datasets_handle.filter_velocity_data(config2.velocity_filter)
datasets_handle.filter_orientation_data(config2.orientation_filter)
datasets_handle.extend_position_data(config2.position_extender)
datasets_handle.extend_orientation_data(config2.orientation_extender)

print("--- %s seconds ---" % (t.time() - start_time))
print("Evaluating ProMPs")

promp_reader = ProMPReader(config.promp_file)

datasets = []
datasets_der = []
phase_se = []
phase_der_se = []

for phase in range(len(promp_reader.promp_handles)):
	dataset = DataSet()
	dataset_der = DataSet()
	dataset_se = DataSet()
	dataset_der_se = DataSet()
	
	z = promp_reader.promp_handles[phase][2]
	t_start, t_end = z.get_extended_start_end()
	t_start_phase, t_end_phase = z.get_phase_start_end()
	print(t_start,t_end, t_start_phase, t_end_phase)
	t_start = float(Decimal(t_start).quantize(Decimal(str(0.1)), ROUND_UP))
	t_end = float(Decimal(t_end).quantize(Decimal(str(0.1)), ROUND_DOWN))
	timerange = np.arange(t_start, t_end, config.step_size).tolist()
	via_points = DataSet()
	for via_point in config.via_points[2]:
		if via_point.time >= t_start and via_point.time <= t_end:
			via_points.append(via_point)
	z.movement_primitive.set_weights_covariance(0.00001)
	data, sigma = z.evaluate(timerange, via_points=via_points)
	data_se, sigma_se = z.evaluate([t_start_phase, t_end_phase], via_points=via_points)
	data_der, sigma_der = z.evaluate(timerange, derivative=1, via_points=via_points)
	data_der_se, sigma_der_se = z.evaluate([t_start_phase, t_end_phase], derivative=1, via_points=via_points)
	
	for i in range(len(timerange)):
		dataset.append(DataPoint(timerange[i], data[i]))
		dataset_der.append(DataPoint(timerange[i], data_der[i]))
	
	dataset_se.append(DataPoint(t_start_phase, data_se[0]))
	dataset_der_se.append(DataPoint(t_start_phase, data_der_se[0]))
	dataset_se.append(DataPoint(t_end_phase, data_se[1]))
	dataset_der_se.append(DataPoint(t_end_phase, data_der_se[1]))
	
	datasets.append(dataset)
	datasets_der.append(dataset_der)
	phase_se.append(dataset_se)
	phase_der_se.append(dataset_der_se)
	
# Align data in time
print("Align data in time")
z_phases = []
zd_phases = []
for phase in range(datasets_handle.n_phases):
	z_phase = []
	zd_phase = []
	for i in range(len(datasets_handle.z_demos)):
		extended_data = datasets_handle.z_demos[i].get_extended_data(phase).copy()
		extended_derivative_data = datasets_handle.z_demos[i].get_extended_derivative(phase).copy()
		promp = promp_reader.promp_handles[phase][2]
		t_start, t_end = promp.get_phase_start_end()
		t_start_extended, t_end_extended = promp.get_extended_start_end()
		if phase == 0:
			# Align to impact at end of phase
			time_alignment = t_end - (extended_data[-1].time - extended_data[0].time) + (t_end_extended - t_end)
		elif phase == datasets_handle.n_phases - 1:
			# Align to impact at start of phase
			time_alignment = t_start + (t_start_extended - t_start)
		else:
			# Align to center
			time_alignment = (t_start + t_end - (extended_data[-1].time - extended_data[0].time)) / 2 + ((t_start_extended - t_start) + (t_end_extended - t_end)) / 2
		extended_data.align_time(time_alignment)
		extended_derivative_data.align_time(time_alignment)
		z_phase.append(extended_data)
		zd_phase.append(extended_derivative_data)
	z_phases.append(z_phase)
	zd_phases.append(zd_phase)

for i in range(len(datasets)):
	plt.figure(figsize=config.figsize,dpi=config.dpi)
	phase_data = datasets[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	for j in range(len(z_phases[i])):
		extended_data = z_phases[i][j]
		plt.plot(extended_data.time, extended_data.value,'C' + str(j+2) + '-*',linewidth=config.linewidth, markersize=config.markersize3,label='Data ' + str(j))
	plt.plot(phase_data.time, phase_data.value,'C1-*',linewidth=config.linewidth, markersize=config.markersize2,label='ProMP')
	plt.plot(phase_se[i].time, phase_se[i].value,'C1*',linewidth=config.linewidth, markersize=config.markersize1,label='Phase ' + str(i) + ' start and end')
	plt.legend(fontsize=config.fontsize2)
	plt.xlabel('Time [s]',fontsize=config.fontsize2)
	plt.ylabel('Position [m]',fontsize=config.fontsize2)
	plt.title('Z position phase ' + str(i),fontsize=config.fontsize1)
	if config.xlim is not None:
		plt.xlim(config.xlim)

for i in range(len(datasets_der)):
	plt.figure(figsize=config.figsize,dpi=config.dpi)
	phase_data = datasets_der[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	for j in range(len(zd_phases[i])):
		extended_data = zd_phases[i][j]
		plt.plot(extended_data.time, extended_data.value,'C' + str(j+2) + '-*',linewidth=config.linewidth, markersize=config.markersize3,label='Data ' + str(j))
	plt.plot(phase_data.time, phase_data.value,'C1-*',linewidth=config.linewidth, markersize=config.markersize2,label='ProMP')
	plt.plot(phase_der_se[i].time, phase_der_se[i].value,'C1*',linewidth=config.linewidth, markersize=config.markersize1,label='Phase ' + str(i) + ' start and end')
	plt.legend(fontsize=config.fontsize2)
	plt.xlabel('Time [s]',fontsize=config.fontsize2)
	plt.ylabel('Velocity [m/s]',fontsize=config.fontsize2)
	plt.title('Z velocity phase ' + str(i),fontsize=config.fontsize1)
	if config.xlim is not None:
		plt.xlim(config.xlim)
	
plt.show()
