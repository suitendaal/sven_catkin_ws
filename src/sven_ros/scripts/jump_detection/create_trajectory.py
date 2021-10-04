#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from models import *
import config.config_create_trajectory as config
import time as t

start_time = t.time()

position_datasets = []
orientation_datasets = []
velocity_datasets = []
rotational_velocity_datasets = []

print("Reading demonstration data")

for demo in config.demos:
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
	
datasets_handle = RobotDataSets(position_datasets, velocity_datasets, orientation_datasets, rotational_velocity_datasets, config.impact_intervals, config.impact_duration)

print("--- %s seconds ---" % (t.time() - start_time))
print("Filtering and extending demonstration data")

datasets_handle.filter_position_data(config.position_filter)
datasets_handle.filter_velocity_data(config.velocity_filter)
datasets_handle.filter_orientation_data(config.orientation_filter)
datasets_handle.extend_position_data(config.position_extender)
datasets_handle.extend_orientation_data(config.orientation_extender)

print("--- %s seconds ---" % (t.time() - start_time))
print("Creating ProMPs")

position_promps = datasets_handle.create_position_promps(config.rbf_width, config.n_rbfs_per_second)
orientation_promps = datasets_handle.create_orientation_promps(config.rbf_width, config.n_rbfs_per_second)

print("--- %s seconds ---" % (t.time() - start_time))		

# Save promps to file
if config.write_mps:
	print("Saving ProMPs to file")
	
	json_object = dict()
	json_object['rotation_matrix'] = datasets_handle.rotation_matrix.tolist()
	json_object['phases'] = []
	for phase in range(datasets_handle.n_phases):
		pos_promp_dicts = []
		or_promp_dicts = []
		for i in range(3):
			pos_promp_dicts.append(position_promps[phase].value[i].to_dict())
			or_promp_dicts.append(orientation_promps[phase].value[i].to_dict())
		phase_promps = dict()
		phase_promps['position_promps'] = pos_promp_dicts
		phase_promps['orientation_promps'] = or_promp_dicts
		json_object['phases'].append(phase_promps)
	
	with open(config.output_file, 'w', encoding='utf-8') as f:
		json.dump(json_object, f, ensure_ascii=False, indent=4)
	
	print("--- %s seconds ---" % (t.time() - start_time))

print("Done")

