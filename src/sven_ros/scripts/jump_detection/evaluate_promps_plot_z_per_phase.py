#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_evaluate_promps as config
import config.config_create_trajectory as config2
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP

# Initialize data arrays
position_data = []
orientation_data = []
velocity_data = []
rotational_velocity_data = []

print("Reading demonstration data")

# Read data of each demo
for demo in config2.demos:
	franka_reader = FrankaStateReader(demo)

	position = []
	orientation = []
	velocity = []
	rotational_velocity = []
	for i in range(3):
		position.append(DataSet())
		orientation.append(DataSet())
		velocity.append(DataSet())
		rotational_velocity.append(DataSet())

	for i in range(len(franka_reader.msgs)):
		dp = franka_reader.next_datapoint()
		time = dp.time
		dp = dp.value
		
		for j in range(3):
			position[j].append(DataPoint(time, dp.position[j]))
			orientation[j].append(DataPoint(time, dp.euler_angles[j]))
			velocity[j].append(DataPoint(time, dp.velocity[j]))
			rotational_velocity[j].append(DataPoint(time, dp.rotational_velocity[j]))
		
	for i in range(3):
		position[i].align_time()
		orientation[i].align_time()
		velocity[i].align_time()
		rotational_velocity[i].align_time()
		
	position_data.append(position)
	orientation_data.append(orientation)
	velocity_data.append(velocity)
	rotational_velocity_data.append(rotational_velocity)
	
print("Filtering and extending demonstration data")

# Initialize robot variables
position_variables = []
orientation_variables = []

# Filter and extend data
for i in range(3):
	position = []
	orientation = []
	velocity = []
	rotational_velocity = []
	
	for j in range(len(config2.demos)):
		position.append(position_data[j][i])
		orientation.append(orientation_data[j][i])
		velocity.append(velocity_data[j][i])
		rotational_velocity.append(rotational_velocity_data[j][i])
	
	position_variable = RobotVariable(position, velocity, config2.impact_intervals)
	position_variable.filter_data(config2.position_filter)
	position_variable.filter_derivative(config2.velocity_filter)
	position_variable.extend_data(config2.position_extender)
	position_variables.append(position_variable)
	
	orientation_variable = RobotVariable(orientation, rotational_velocity, config2.impact_intervals)
	orientation_variable.filter_data(config2.orientation_filter)
	orientation_variable.filter_derivative(config2.rotational_velocity_filter)
	orientation_variable.extend_data(config2.orientation_extender)
	orientation_variables.append(orientation_variable)

print("Evaluating ProMPs")

promp_reader = ProMPReader(config.promp_file)

datasets = []
datasets_der = []

for phase in range(len(promp_reader.promp_handles)):
	dataset = DataSet()
	dataset_der = DataSet()
	z = promp_reader.promp_handles[phase][2]
	t_start, t_end = z.get_extended_start_end()
	t_start = float(Decimal(t_start).quantize(Decimal(str(0.1)), ROUND_UP))
	t_end = float(Decimal(t_end).quantize(Decimal(str(0.1)), ROUND_DOWN))
	timerange = np.arange(t_start, t_end, 0.1).tolist()
	via_points = DataSet()
	for via_point in config.via_points[2]:
		if via_point.time >= t_start and via_point.time <= t_end:
			via_points.append(via_point)
	z.movement_primitive.set_weights_covariance(0.00001)
	data, sigma = z.evaluate(timerange, via_points=via_points)
	data_der, sigma_der = z.evaluate(timerange, derivative=1, via_points=via_points)
	
	for i in range(len(timerange)):
		dataset.append(DataPoint(timerange[i], data[i]))
		dataset_der.append(DataPoint(timerange[i], data_der[i]))
	datasets.append(dataset)
	datasets_der.append(dataset_der)
	
# Align data in time
print("Align data in time")
z_variable = position_variables[2]
z_phases = []
zd_phases = []
for phase in range(z_variable.n_phases):
	z_phase = []
	zd_phase = []
	for i in range(len(z_variable.demo_variables)):
		extended_data = z_variable.demo_variables[i].get_extended_data(phase).copy()
		extended_derivative_data = z_variable.demo_variables[i].get_extended_derivative(phase).copy()
		promp = promp_reader.promp_handles[phase][2]
		t_start, t_end = promp.get_phase_start_end()
		t_start_extended, t_end_extended = promp.get_extended_start_end()
		if phase == 0:
			# Align to impact at end of phase
			time_alignment = t_end - (extended_data[-1].time - extended_data[0].time) + (t_end_extended - t_end)
		elif phase == z_variable.n_phases - 1:
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
	plt.legend(fontsize=config.fontsize2)
	plt.xlabel('Time [s]',fontsize=config.fontsize2)
	plt.ylabel('Position [m]',fontsize=config.fontsize2)
	plt.title('Z position phase ' + str(i),fontsize=config.fontsize1)

for i in range(len(datasets_der)):
	plt.figure(figsize=config.figsize,dpi=config.dpi)
	phase_data = datasets_der[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	for j in range(len(zd_phases[i])):
		extended_data = zd_phases[i][j]
		plt.plot(extended_data.time, extended_data.value,'C' + str(j+2) + '-*',linewidth=config.linewidth, markersize=config.markersize3,label='Data ' + str(j))
	plt.plot(phase_data.time, phase_data.value,'C1-*',linewidth=config.linewidth, markersize=config.markersize2,label='ProMP')
	plt.legend(fontsize=config.fontsize2)
	plt.xlabel('Time [s]',fontsize=config.fontsize2)
	plt.ylabel('Velocity [m/s]',fontsize=config.fontsize2)
	plt.title('Z velocity phase ' + str(i),fontsize=config.fontsize1)
	
plt.show()
