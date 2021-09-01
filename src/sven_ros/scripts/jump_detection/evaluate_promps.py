#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_evaluate_promps as config
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP

def round_up(number, step_size):
	return float(Decimal(number).quantize(Decimal(str(step_size)), ROUND_UP))
	
def round_down(number, step_size):
	return float(Decimal(number).quantize(Decimal(str(step_size)), ROUND_DOWN))

promp_reader = ProMPReader(config.promp_file)

datasets = []
datasets_derivative = []

for phase in range(len(promp_reader.promp_handles)):
	datasets_phase = []
	datasets_derivative_phase = []
	
	for i in range(len(promp_reader.promp_handles[phase])):
		promp_handle = promp_reader.promp_handles[phase][i]
		dataset_phase = DataSet()
		dataset_derivative_phase = DataSet()
		
		# Time range for phase
		t_start, t_end = promp_handle.get_extended_start_end()
		t_start = round_up(t_start, config.step_size)
		t_end = round_up(t_end, config.step_size)
		time = np.arange(t_start, t_end, config.step_size).tolist()
		
		# Via points for phase
		via_points = DataSet()
		for via_point in config.via_points[i]:
			if via_point.time >= t_start and via_point.time <= t_end:
				via_points.append(via_point)
				
		# Evaluate promp
		data, sigma = promp_handle.evaluate(time, via_points=via_points)
		data_der, sigma_der = promp_handle.evaluate(time, derivative=1, via_points=via_points)
		for i in range(len(time)):
			dataset_phase.append(DataPoint(time[i], data[i]))
			dataset_derivative_phase.append(DataPoint(time[i], data_der[i]))
		datasets_phase.append(dataset_phase)
		datasets_derivative_phase.append(dataset_derivative_phase)
	
	datasets.append(datasets_phase)
	datasets_derivative.append(datasets_derivative_phase)
	
if config.write_evaluation:
	data = dict()
	data['datasets'] = []
	for datasets_phase in datasets:
		datasets_phase_dict = []
		for i in range(len(datasets_phase)):
			dataset = datasets_phase[i]
			dataset_dict = dict()
			dataset_dict['label'] = config.variable_labels[i]
			dataset_dict['time'] = dataset.time
			dataset_dict['value'] = dataset.value
			datasets_phase_dict.append(dataset_dict)
		data['datasets'].append(datasets_phase_dict)
	data['datasets_derivative'] = []
	for datasets_derivative_phase in datasets_derivative:
		datasets_derivative_phase_dict = []
		for i in range(len(datasets_derivative_phase)):
			dataset = datasets_derivative_phase[i]
			dataset_dict = dict()
			dataset_dict['label'] = config.variable_labels[i]
			dataset_dict['time'] = dataset.time
			dataset_dict['value'] = dataset.value
			datasets_derivative_phase_dict.append(dataset_dict)
		data['datasets_derivative'].append(datasets_derivative_phase_dict)
	
	with open(config.output_file, 'w', encoding='utf-8') as f:
		json.dump(data, f, ensure_ascii=False, indent=4)
	
if config.plot_figs:
	for i in range(len(datasets[0])):
		plt.figure()
		for phase in range(len(datasets)):
			phase_data = datasets[phase][i]
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2
			plt.plot(phase_data.time, phase_data.value,'C' + str(phase) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='Phase ' + str(phase))
		plt.legend()
		plt.title('Position ' + config.variable_labels[i])
		
		plt.figure()
		for phase in range(len(datasets_derivative)):
			phase_data = datasets_derivative[phase][i]
			plt.rcParams['xtick.labelsize'] = config.fontsize2
			plt.rcParams['ytick.labelsize'] = config.fontsize2
			plt.plot(phase_data.time, phase_data.value,'C' + str(phase) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='Phase ' + str(phase))
		plt.legend()
		plt.title('Velocity ' + config.variable_labels[i])
	
	plt.show()

