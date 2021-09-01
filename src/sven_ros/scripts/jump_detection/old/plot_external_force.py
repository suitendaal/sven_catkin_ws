#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *

# Define datafile
bagfile = 'data/replay4.1.bag'
franka_reader = FrankaStateReader(bagfile)

# Figure settings
figsize = (16, 12)
dpi = 80
linewidth = 1
markersize1 = 3
markersize2 = 10
fontsize1 = 20
fontsize2 = 16
xlim = None
#xlim = [8.6, 8.8]
#xlim = [8.5, 11]
labels = ['X','Y','Z']
save_figs = False
show_figs = True
figure_dir = 'figures/external_force_plots'

# Initialize datasets
tau = []
tau_ext = []
for i in range(7):
	tau.append(DataSet())
	tau_ext.append(DataSet())

force = []
force_ext = []
abs_force_ext = DataSet()
for i in range(3):
	force.append(DataSet())
	force_ext.append(DataSet())

# Read datasets
for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	time = dp.time
	value = dp.value
	
	for j in range(7):
		tau[j].append(DataPoint(time, value.tau_measured[j]))
		tau_ext[j].append(DataPoint(time, value.tau_external[j]))
		
	for j in range(3):
		force[j].append(DataPoint(time, value.force_measured[j]))
		force_ext[j].append(DataPoint(time, value.force_external[j]))
		
	abs_force_ext.append(DataPoint(time, np.linalg.norm(value.force_external)))

## Align time
#for i in range(7):
#	tau_ext[i].align_time()

#for i in range(3):
#	force_ext[i].align_time()
		
## External joint torque		
#fig = plt.figure(figsize=figsize, dpi=dpi)
#plt.rcParams['xtick.labelsize']=fontsize2
#plt.rcParams['ytick.labelsize']=fontsize2
#for i in range(7):
#	plt.plot(tau_ext[i].time, tau_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
#	
#plt.title('External joint torque',fontsize=fontsize1)
#plt.xlabel('Time [s]',fontsize=fontsize2)
#plt.ylabel('Torque [Nm]',fontsize=fontsize2)
#plt.legend(fontsize=fontsize2)
#if xlim is not None:
#	plt.xlim(xlim)
#	
#if save_figs:
#	title = figure_dir + '/' + fig.axes[0].get_title()
#	if xlim is not None:
#		title += 'xlim'
#	title += '.png'
#	plt.savefig(title)
#if not show_figs:
#	plt.close()
#	
## Joint torque		
#fig = plt.figure(figsize=figsize, dpi=dpi)
#plt.rcParams['xtick.labelsize']=fontsize2
#plt.rcParams['ytick.labelsize']=fontsize2
#for i in range(7):
#	plt.plot(tau[i].time, tau[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
#	
#plt.title('Joint torque',fontsize=fontsize1)
#plt.xlabel('Time [s]',fontsize=fontsize2)
#plt.ylabel('Torque [Nm]',fontsize=fontsize2)
#plt.legend(fontsize=fontsize2)
#if xlim is not None:
#	plt.xlim(xlim)
#	
#if save_figs:
#	title = figure_dir + '/' + fig.axes[0].get_title()
#	if xlim is not None:
#		title += 'xlim'
#	title += '.png'
#	plt.savefig(title)
#if not show_figs:
#	plt.close()

## End effector external force	
#fig = plt.figure(figsize=figsize, dpi=dpi)
#plt.rcParams['xtick.labelsize']=fontsize2
#plt.rcParams['ytick.labelsize']=fontsize2
#for i in range(3):
#	plt.plot(force_ext[i].time, force_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])

#plt.title('End effector external force',fontsize=fontsize1)
#plt.xlabel('Time [s]',fontsize=fontsize2)
#plt.ylabel('Force [N]',fontsize=fontsize2)
#plt.legend(fontsize=fontsize2)
#if xlim is not None:
#	plt.xlim(xlim)
#	
#if save_figs:
#	title = figure_dir + '/' + fig.axes[0].get_title()
#	if xlim is not None:
#		title += 'xlim'
#	title += '.png'
#	plt.savefig(title)
#if not show_figs:
#	plt.close()
#	
## End effector force	
#fig = plt.figure(figsize=figsize, dpi=dpi)
#plt.rcParams['xtick.labelsize']=fontsize2
#plt.rcParams['ytick.labelsize']=fontsize2
#for i in range(3):
#	plt.plot(force[i].time, force[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])

#plt.title('End effector force',fontsize=fontsize1)
#plt.xlabel('Time [s]',fontsize=fontsize2)
#plt.ylabel('Force [N]',fontsize=fontsize2)
#plt.legend(fontsize=fontsize2)
#if xlim is not None:
#	plt.xlim(xlim)
#	
#if save_figs:
#	title = figure_dir + '/' + fig.axes[0].get_title()
#	if xlim is not None:
#		title += 'xlim'
#	title += '.png'
#	plt.savefig(title)
#if not show_figs:
#	plt.close()

# Apply 2nd order ja filter
bounder = ConstantBounder(bound=6)
predictor = LeastSquaresFilter(window_length=20, order=2)
ja_filter = JumpAwareFilter(predictor, bounder)
abs_force_ext_ji = []
abs_force_ext_pred = DataSet()

# External force magnitude
print("Analyzing external force magnitude")
data = abs_force_ext
jump_indices = abs_force_ext_ji
predictions = abs_force_ext_pred
ja_filter.reset()
for j in range(len(data)):
	jump_detected, info = ja_filter.update(data[j])
	if jump_detected:
		jump_indices.append(j)
	predictions.append(info[0])
	
# Plot data
# End effector external force magnitude
abs_force_ext_j = DataSet()
force_ext_j = []
for i in range(3):
	force_ext_j.append(DataSet())
data = abs_force_ext
jumping_indices = abs_force_ext_ji
jumps = abs_force_ext_j
for j in jumping_indices:
	jumps.append(data[j])
	for i in range(3):
		force_ext_j[i].append(force_ext[i][j])

abs_force_ext_pred_j = DataSet()
data = abs_force_ext_pred
jumping_indices = abs_force_ext_ji
jumps = abs_force_ext_pred_j
for j in jumping_indices:
	jumps.append(data[j])
	
abs_force_ext_bounds = DataSet()
for j in abs_force_ext:
	abs_force_ext_bounds.append(DataPoint(j.time, 6))

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_force_ext.time, abs_force_ext.value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_force_ext_j.time, abs_force_ext_j.value, 'C1*', markersize=markersize2)
plt.plot(abs_force_ext_pred.time, abs_force_ext_pred.value, 'C2-*', linewidth=linewidth, markersize=markersize1,label='Prediction')

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_force_ext.time, abs(abs_force_ext-abs_force_ext_pred).value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_force_ext_j.time, abs(abs_force_ext_j-abs_force_ext_pred_j).value, 'C1*', markersize=markersize2)
plt.plot(abs_force_ext_bounds.time, abs_force_ext_bounds.value,'C7-',linewidth=linewidth,label='Bound')


if show_figs:
	plt.show()
	
print(abs_force_ext_j)
print(abs_force_ext_pred_j)
for i in range(3):
	print(force_ext_j[i])

