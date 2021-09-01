#!/usr/bin/python3

from readers import *
from datalib import *
from models import *
import config.config_create_trajectory as config
import json

# Initialize data arrays
position_data = []
orientation_data = []
velocity_data = []
rotational_velocity_data = []

print("Reading demonstration data")

# Read data of each demo
for demo in config.demos:
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
	
	for j in range(len(config.demos)):
		position.append(position_data[j][i])
		orientation.append(orientation_data[j][i])
		velocity.append(velocity_data[j][i])
		rotational_velocity.append(rotational_velocity_data[j][i])
	
	position_variable = RobotVariable(position, velocity, config.impact_intervals)
	position_variable.filter_data(config.position_filter)
	position_variable.filter_derivative(config.velocity_filter)
	position_variable.extend_data(config.position_extender)
	position_variables.append(position_variable)
	
	orientation_variable = RobotVariable(orientation, rotational_velocity, config.impact_intervals)
	orientation_variable.filter_data(config.orientation_filter)
	orientation_variable.filter_derivative(config.rotational_velocity_filter)
	orientation_variable.extend_data(config.orientation_extender)
	orientation_variables.append(orientation_variable)
	
print("Creating ProMPs")
	
# Create promps
position_promps = []
orientation_promps = []
for i in range(position_variables[0].n_phases):
	position_promps.append([])
	orientation_promps.append([])

for pos_var in position_variables:
	promps = pos_var.create_promps(config.rbf_width, config.n_rbfs_per_second)
	for phase in range(len(promps)):
		position_promps[phase].append(promps[phase])
for or_var in orientation_variables:
	promps = or_var.create_promps(config.rbf_width, config.n_rbfs_per_second)
	for phase in range(len(promps)):
		orientation_promps[phase].append(promps[phase])
		
print("Saving ProMPs to file")
		
# Save promps to file
if config.write_mps:
	json_object = dict()
	json_object['phases'] = []
	for phase in range(len(position_promps)):
		pos_promp_dicts = []
		or_promp_dicts = []
		for i in range(3):
			pos_promp_dicts.append(position_promps[phase][i].to_dict())
			or_promp_dicts.append(orientation_promps[phase][i].to_dict())
		phase_promps = dict()
		phase_promps['position_promps'] = pos_promp_dicts
		phase_promps['orientation_promps'] = or_promp_dicts
		json_object['phases'].append(phase_promps)
	
	with open(config.output_file, 'w', encoding='utf-8') as f:
		json.dump(json_object, f, ensure_ascii=False, indent=4)
		
print("Done")


