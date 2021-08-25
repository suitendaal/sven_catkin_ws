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
save_figs = True
show_figs = False
figure_dir = 'figures/external_force_plots'

# Initialize datasets
tau = []
tau_ext = []
for i in range(7):
	tau.append(DataSet())
	tau_ext.append(DataSet())

force = []
force_ext = []
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

# Align time
for i in range(7):
	tau_ext[i].align_time()

for i in range(3):
	force_ext[i].align_time()
		
# External joint torque		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau_ext[i].time, tau_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	
plt.title('External joint torque',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Torque [Nm]',fontsize=fontsize2)
plt.legend(fontsize=fontsize2)
if xlim is not None:
	plt.xlim(xlim)
	
if save_figs:
	title = figure_dir + '/' + fig.axes[0].get_title()
	if xlim is not None:
		title += 'xlim'
	title += '.png'
	plt.savefig(title)
if not show_figs:
	plt.close()
	
# Joint torque		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau[i].time, tau[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	
plt.title('Joint torque',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Torque [Nm]',fontsize=fontsize2)
plt.legend(fontsize=fontsize2)
if xlim is not None:
	plt.xlim(xlim)
	
if save_figs:
	title = figure_dir + '/' + fig.axes[0].get_title()
	if xlim is not None:
		title += 'xlim'
	title += '.png'
	plt.savefig(title)
if not show_figs:
	plt.close()

# End effector external force	
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force_ext[i].time, force_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])

plt.title('End effector external force',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Force [N]',fontsize=fontsize2)
plt.legend(fontsize=fontsize2)
if xlim is not None:
	plt.xlim(xlim)
	
if save_figs:
	title = figure_dir + '/' + fig.axes[0].get_title()
	if xlim is not None:
		title += 'xlim'
	title += '.png'
	plt.savefig(title)
if not show_figs:
	plt.close()
	
# End effector force	
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force[i].time, force[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])

plt.title('End effector force',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Force [N]',fontsize=fontsize2)
plt.legend(fontsize=fontsize2)
if xlim is not None:
	plt.xlim(xlim)
	
if save_figs:
	title = figure_dir + '/' + fig.axes[0].get_title()
	if xlim is not None:
		title += 'xlim'
	title += '.png'
	plt.savefig(title)
if not show_figs:
	plt.close()

if show_figs:
	plt.show()

