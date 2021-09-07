#!/usr/bin/python3

from readers import *
from models import *
from datalib import *
import config.config_evaluate_promps as config
import matplotlib.pyplot as plt
from decimal import Decimal, ROUND_DOWN, ROUND_UP

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

plt.figure(figsize=config.figsize,dpi=config.dpi)
for i in range(len(datasets)):
	phase_data = datasets[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	plt.plot(phase_data.time, phase_data.value,'C' + str(i) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='ProMP phase ' + str(i))
plt.legend(fontsize=config.fontsize2)
plt.xlabel('Time [s]',fontsize=config.fontsize2)
plt.ylabel('Position [m]',fontsize=config.fontsize2)
plt.title('Z position',fontsize=config.fontsize1)

plt.figure(figsize=config.figsize,dpi=config.dpi)
for i in range(len(datasets_der)):
	phase_data = datasets_der[i]
	plt.rcParams['xtick.labelsize'] = config.fontsize2
	plt.rcParams['ytick.labelsize'] = config.fontsize2
	plt.plot(phase_data.time, phase_data.value,'C' + str(i) + '-*',linewidth=config.linewidth, markersize=config.markersize2,label='ProMP phase ' + str(i))
plt.legend(fontsize=config.fontsize2)
plt.xlabel('Time [s]',fontsize=config.fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=config.fontsize2)
plt.title('Z velocity',fontsize=config.fontsize1)
	
plt.show()
