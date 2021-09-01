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
# Full trajectory
xlim = None
# Zoomed in on first impact
#xlim = [8.6, 8.8]
# Zoomed in on impact interval
#xlim = [8.6, 9.4]
# Zoomed in on high force
#xlim = [8.5, 11]
labels = ['X','Y','Z']
save_figs = False
show_figs = True
figure_dir = 'figures/different_plots'

# Initialize datasets
q = []
dq = []
tau = []
tau_ext = []
for i in range(7):
	q.append(DataSet())
	dq.append(DataSet())
	tau.append(DataSet())
	tau_ext.append(DataSet())

position = []
velocity = []
force = []
force_ext = []
for i in range(3):
	position.append(DataSet())
	velocity.append(DataSet())
	force.append(DataSet())
	force_ext.append(DataSet())

abs_velocity = DataSet()
abs_force_ext = DataSet()

# Initialize bounds
q_bound = 0.0018
dq_bound = 0.1
tau_bound = 3
tau_ext_bound = 1.5
position_bound = 0.0007
velocity_bound = 0.03
force_bound = 10
force_ext_bound = 6
abs_velocity_bound = 0.03
abs_force_ext_bound = 6
force_ext_zero_bound = 70
abs_force_ext_zero_bound = 70

# Read datasets
for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	time = dp.time
	value = dp.value
	
	for j in range(7):
		q[j].append(DataPoint(time, value.q[j]))
		dq[j].append(DataPoint(time, value.dq[j]))
		tau[j].append(DataPoint(time, value.tau_measured[j]))
		tau_ext[j].append(DataPoint(time, value.tau_external[j]))
		
	for j in range(3):
		position[j].append(DataPoint(time, value.position[j]))
		velocity[j].append(DataPoint(time, value.velocity[j]))
		force[j].append(DataPoint(time, value.force_measured[j]))
		force_ext[j].append(DataPoint(time, value.force_external[j]))
	
	abs_velocity.append(DataPoint(time, np.linalg.norm(value.velocity)))
	abs_force_ext.append(DataPoint(time, np.linalg.norm(value.force_external)))

# Align time
for i in range(7):
	q[i].align_time()
	dq[i].align_time()
	tau[i].align_time()
	tau_ext[i].align_time()

for i in range(3):
	position[i].align_time()
	velocity[i].align_time()
	force[i].align_time()
	force_ext[i].align_time()

abs_velocity.align_time()
abs_force_ext.align_time()

# Initialize results of ja filter
q_ji = []
q_pred = []
dq_ji = []
dq_pred = []
tau_ji = []
tau_pred = []
tau_ext_ji = []
tau_ext_pred = []
position_ji = []
position_pred = []
velocity_ji = []
velocity_pred = []
force_ji = []
force_pred = []
force_ext_ji = []
force_ext_pred = []
force_ext_zero_ji = []
force_ext_zero_pred = []
abs_velocity_ji = []
abs_velocity_pred = DataSet()
abs_force_ext_ji = []
abs_force_ext_pred = DataSet()
abs_force_ext_zero_ji = []
abs_force_ext_zero_pred = DataSet()

# Apply 2nd order ja filter
bounder = ConstantBounder(bound=0.1)
predictor = LeastSquaresFilter(window_length=20, order=2)
ja_filter = JumpAwareFilter(predictor, bounder)

# Joint position
print("Analyzing joint position")
predictor.order = 2
bounder.set_bound(q_bound)
for i in range(7):
	q_ji.append([])
	q_pred.append(DataSet())
for i in range(7):
	data = q[i]
	jump_indices = q_ji[i]
	predictions = q_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Joint velocity
print("Analyzing joint velocity")
predictor.order = 2
bounder.set_bound(dq_bound)
for i in range(7):
	dq_ji.append([])
	dq_pred.append(DataSet())
for i in range(7):
	data = dq[i]
	jump_indices = dq_ji[i]
	predictions = dq_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Joint torque
print("Analyzing joint torque")
predictor.order = 2
bounder.set_bound(tau_bound)
for i in range(7):
	tau_ji.append([])
	tau_pred.append(DataSet())
for i in range(7):
	data = tau[i]
	jump_indices = tau_ji[i]
	predictions = tau_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# External joint torque
print("Analyzing external joint torque")
predictor.order = 2
bounder.set_bound(tau_ext_bound)
for i in range(7):
	tau_ext_ji.append([])
	tau_ext_pred.append(DataSet())
for i in range(7):
	data = tau_ext[i]
	jump_indices = tau_ext_ji[i]
	predictions = tau_ext_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Position
print("Analyzing position")
predictor.order = 2
bounder.set_bound(position_bound)
for i in range(3):
	position_ji.append([])
	position_pred.append(DataSet())
for i in range(3):
	data = position[i]
	jump_indices = position_ji[i]
	predictions = position_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Velocity
print("Analyzing velocity")
predictor.order = 2
bounder.set_bound(velocity_bound)
for i in range(3):
	velocity_ji.append([])
	velocity_pred.append(DataSet())
for i in range(3):
	data = velocity[i]
	jump_indices = velocity_ji[i]
	predictions = velocity_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Force
print("Analyzing force")
predictor.order = 2
bounder.set_bound(force_bound)
for i in range(3):
	force_ji.append([])
	force_pred.append(DataSet())
for i in range(3):
	data = force[i]
	jump_indices = force_ji[i]
	predictions = force_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# External force
print("Analyzing external force")
predictor.order = 2
bounder.set_bound(force_ext_bound)
for i in range(3):
	force_ext_ji.append([])
	force_ext_pred.append(DataSet())
for i in range(3):
	data = force_ext[i]
	jump_indices = force_ext_ji[i]
	predictions = force_ext_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])
		
# Velocity magnitude
print("Analyzing velocity magnitude")
predictor.order = 2
bounder.set_bound(abs_velocity_bound)
data = abs_velocity
jump_indices = abs_velocity_ji
predictions = abs_velocity_pred
ja_filter.reset()
for j in range(len(data)):
	jump_detected, info = ja_filter.update(data[j])
	if jump_detected:
		jump_indices.append(j)
	predictions.append(info[0])
	
# External force magnitude
print("Analyzing external force magnitude")
predictor.order = 2
bounder.set_bound(abs_force_ext_bound)
data = abs_force_ext
jump_indices = abs_force_ext_ji
predictions = abs_force_ext_pred
ja_filter.reset()
for j in range(len(data)):
	jump_detected, info = ja_filter.update(data[j])
	if jump_detected:
		jump_indices.append(j)
	predictions.append(info[0])
	
# External force zero order
print("Analyzing external force using zero order predictions")
ja_filter.predictor = ConstantFilter()
predictor.order = 0
bounder.set_bound(force_ext_zero_bound)
for i in range(3):
	force_ext_zero_ji.append([])
	force_ext_zero_pred.append(DataSet())
for i in range(3):
	data = force_ext[i]
	jump_indices = force_ext_zero_ji[i]
	predictions = force_ext_zero_pred[i]
	ja_filter.reset()
	for j in range(len(data)):
		jump_detected, info = ja_filter.update(data[j])
		if jump_detected:
			jump_indices.append(j)
		predictions.append(info[0])

# External force magnitude zero order
print("Analyzing external force magnitude using zero order predictions")
predictor.order = 0
bounder.set_bound(abs_force_ext_zero_bound)
data = abs_force_ext
jump_indices = abs_force_ext_zero_ji
predictions = abs_force_ext_zero_pred
ja_filter.reset()
for j in range(len(data)):
	jump_detected, info = ja_filter.update(data[j])
	if jump_detected:
		jump_indices.append(j)
	predictions.append(info[0])

# Joint position
q_j = []
for i in range(7):
	q_j.append(DataSet())
	data = q[i]
	jumping_indices = q_ji[i]
	jumps = q_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
q_pred_j = []
for i in range(7):
	q_pred_j.append(DataSet())
	data = q_pred[i]
	jumping_indices = q_ji[i]
	jumps = q_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
q_bounds = DataSet()
for j in q[0]:
	q_bounds.append(DataPoint(j.time, q_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(q[i].time, (q[i]-q[i][0]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(q_j[i].time, (q_j[i]-q[i][0]).value, 'C' + str(i) + '*', markersize=markersize2)

plt.title('Joint position',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [rad]',fontsize=fontsize2)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(q[i].time, abs(q[i]-q_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(q_j[i].time, abs(q_j[i]-q_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(q_bounds.time, q_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between joint position and prediction',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [rad]',fontsize=fontsize2)
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

# Joint velocity
dq_j = []
for i in range(7):
	dq_j.append(DataSet())
	data = dq[i]
	jumping_indices = dq_ji[i]
	jumps = dq_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
dq_pred_j = []
for i in range(7):
	dq_pred_j.append(DataSet())
	data = dq_pred[i]
	jumping_indices = dq_ji[i]
	jumps = dq_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
dq_bounds = DataSet()
for j in dq[0]:
	dq_bounds.append(DataPoint(j.time, dq_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(dq[i].time, dq[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(dq_j[i].time, dq_j[i].value, 'C' + str(i) + '*', markersize=markersize2)
	
plt.title('Joint velocity',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [rad/s]',fontsize=fontsize2)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(dq[i].time, abs(dq[i]-dq_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(dq_j[i].time, abs(dq_j[i]-dq_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(dq_bounds.time, dq_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between joint velocity and prediction',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [rad/s]',fontsize=fontsize2)
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
tau_j = []
for i in range(7):
	tau_j.append(DataSet())
	data = tau[i]
	jumping_indices = tau_ji[i]
	jumps = tau_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
tau_pred_j = []
for i in range(7):
	tau_pred_j.append(DataSet())
	data = tau_pred[i]
	jumping_indices = tau_ji[i]
	jumps = tau_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
tau_bounds = DataSet()
for j in q[0]:
	tau_bounds.append(DataPoint(j.time, tau_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau[i].time, tau[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(tau_j[i].time, tau_j[i].value, 'C' + str(i) + '*', markersize=markersize2)

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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau[i].time, abs(tau[i]-tau_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(tau_j[i].time, abs(tau_j[i]-tau_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(tau_bounds.time, tau_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between joint torque and prediction',fontsize=fontsize1)
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
	
# External joint torque
tau_ext_j = []
for i in range(7):
	tau_ext_j.append(DataSet())
	data = tau_ext[i]
	jumping_indices = tau_ext_ji[i]
	jumps = tau_ext_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
tau_ext_pred_j = []
for i in range(7):
	tau_ext_pred_j.append(DataSet())
	data = tau_ext_pred[i]
	jumping_indices = tau_ext_ji[i]
	jumps = tau_ext_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
tau_ext_bounds = DataSet()
for j in q[0]:
	tau_ext_bounds.append(DataPoint(j.time, tau_ext_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau_ext[i].time, tau_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(tau_ext_j[i].time, tau_ext_j[i].value, 'C' + str(i) + '*', markersize=markersize2)

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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(7):
	plt.plot(tau_ext[i].time, abs(tau_ext[i]-tau_ext_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label='Joint ' + str(i+1))
	plt.plot(tau_ext_j[i].time, abs(tau_ext_j[i]-tau_ext_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(tau_ext_bounds.time, tau_ext_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between external joint torque and prediction',fontsize=fontsize1)
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
	
# End effector position
position_j = []
for i in range(3):
	position_j.append(DataSet())
	data = position[i]
	jumping_indices = position_ji[i]
	jumps = position_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
position_pred_j = []
for i in range(3):
	position_pred_j.append(DataSet())
	data = position_pred[i]
	jumping_indices = position_ji[i]
	jumps = position_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
position_bounds = DataSet()
for j in q[0]:
	position_bounds.append(DataPoint(j.time, position_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(position[i].time, (position[i]-position[i][0]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(position_j[i].time, (position_j[i]-position[i][0]).value, 'C' + str(i) + '*', markersize=markersize2)

plt.title('End effector position',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [m]',fontsize=fontsize2)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(position[i].time, abs(position[i]-position_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(position_j[i].time, abs(position_j[i]-position_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(position_bounds.time, position_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector position and prediction',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Position [m]',fontsize=fontsize2)
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
	
# End effector velocity
velocity_j = []
for i in range(3):
	velocity_j.append(DataSet())
	data = velocity[i]
	jumping_indices = velocity_ji[i]
	jumps = velocity_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
velocity_pred_j = []
for i in range(3):
	velocity_pred_j.append(DataSet())
	data = velocity_pred[i]
	jumping_indices = velocity_ji[i]
	jumps = velocity_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
velocity_bounds = DataSet()
for j in q[0]:
	velocity_bounds.append(DataPoint(j.time, velocity_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(velocity[i].time, velocity[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(velocity_j[i].time, velocity_j[i].value, 'C' + str(i) + '*', markersize=markersize2)

plt.title('End effector velocity',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(velocity[i].time, abs(velocity[i]-velocity_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(velocity_j[i].time, abs(velocity_j[i]-velocity_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(velocity_bounds.time, velocity_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector velocity and prediction',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
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
force_j = []
for i in range(3):
	force_j.append(DataSet())
	data = force[i]
	jumping_indices = force_ji[i]
	jumps = force_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
force_pred_j = []
for i in range(3):
	force_pred_j.append(DataSet())
	data = force_pred[i]
	jumping_indices = force_ji[i]
	jumps = force_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
force_bounds = DataSet()
for j in q[0]:
	force_bounds.append(DataPoint(j.time, force_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force[i].time, force[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(force_j[i].time, force_j[i].value, 'C' + str(i) + '*', markersize=markersize2)

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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force[i].time, abs(force[i]-force_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(force_j[i].time, abs(force_j[i]-force_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(force_bounds.time, force_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector force and prediction',fontsize=fontsize1)
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
	
# End effector external force
force_ext_j = []
for i in range(3):
	force_ext_j.append(DataSet())
	data = force_ext[i]
	jumping_indices = force_ext_ji[i]
	jumps = force_ext_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
force_ext_pred_j = []
for i in range(3):
	force_ext_pred_j.append(DataSet())
	data = force_ext_pred[i]
	jumping_indices = force_ext_ji[i]
	jumps = force_ext_pred_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
force_ext_bounds = DataSet()
for j in q[0]:
	force_ext_bounds.append(DataPoint(j.time, force_ext_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force_ext[i].time, force_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(force_ext_j[i].time, force_ext_j[i].value, 'C' + str(i) + '*', markersize=markersize2)

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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force_ext[i].time, abs(force_ext[i]-force_ext_pred[i]).value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(force_ext_j[i].time, abs(force_ext_j[i]-force_ext_pred_j[i]).value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(force_ext_bounds.time, force_ext_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector external force and prediction',fontsize=fontsize1)
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
	
# End effector velocity magnitude
abs_velocity_j = DataSet()
data = abs_velocity
jumping_indices = abs_velocity_ji
jumps = abs_velocity_j
for j in jumping_indices:
	jumps.append(data[j])

abs_velocity_pred_j = DataSet()
data = abs_velocity_pred
jumping_indices = abs_velocity_ji
jumps = abs_velocity_pred_j
for j in jumping_indices:
	jumps.append(data[j])
abs_velocity_bounds = DataSet()
for j in q[0]:
	abs_velocity_bounds.append(DataPoint(j.time, abs_velocity_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_velocity.time, abs_velocity.value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_velocity_j.time, abs_velocity_j.value, 'C1*', markersize=markersize2)

plt.title('End effector velocity magnitude',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_velocity.time, abs(abs_velocity-abs_velocity_pred).value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_velocity_j.time, abs(abs_velocity_j-abs_velocity_pred_j).value, 'C1*', markersize=markersize2)
plt.plot(abs_velocity_bounds.time, abs_velocity_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector velocity magnitude and prediction',fontsize=fontsize1)
plt.xlabel('Time [s]',fontsize=fontsize2)
plt.ylabel('Velocity [m/s]',fontsize=fontsize2)
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
	
# End effector external force magnitude
abs_force_ext_j = DataSet()
data = abs_force_ext
jumping_indices = abs_force_ext_ji
jumps = abs_force_ext_j
for j in jumping_indices:
	jumps.append(data[j])

abs_force_ext_pred_j = DataSet()
data = abs_force_ext_pred
jumping_indices = abs_force_ext_ji
jumps = abs_force_ext_pred_j
for j in jumping_indices:
	jumps.append(data[j])
abs_force_ext_bounds = DataSet()
for j in q[0]:
	abs_force_ext_bounds.append(DataPoint(j.time, abs_force_ext_bound))

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_force_ext.time, abs_force_ext.value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_force_ext_j.time, abs_force_ext_j.value, 'C1*', markersize=markersize2)

plt.title('End effector external force magnitude',fontsize=fontsize1)
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

fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_force_ext.time, abs(abs_force_ext-abs_force_ext_pred).value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_force_ext_j.time, abs(abs_force_ext_j-abs_force_ext_pred_j).value, 'C1*', markersize=markersize2)
plt.plot(abs_force_ext_bounds.time, abs_force_ext_bounds.value,'C7-',linewidth=linewidth,label='Bound')

plt.title('Absolute difference between end effector external force magnitude and prediction',fontsize=fontsize1)
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
	
# End effector external force zero order
force_ext_zero_j = []
for i in range(3):
	force_ext_zero_j.append(DataSet())
	data = force_ext[i]
	jumping_indices = force_ext_zero_ji[i]
	jumps = force_ext_zero_j[i]
	for j in jumping_indices:
		jumps.append(data[j])
force_ext_zero_bounds = DataSet()
for j in q[0]:
	force_ext_zero_bounds.append(DataPoint(j.time, force_ext_zero_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
for i in range(3):
	plt.plot(force_ext[i].time, force_ext[i].value, 'C' + str(i) + '-*', linewidth=linewidth, markersize=markersize1,label=labels[i])
	plt.plot(force_ext_zero_j[i].time, force_ext_zero_j[i].value, 'C' + str(i) + '*', markersize=markersize2)
plt.plot(force_ext_zero_bounds.time, force_ext_zero_bounds.value,'C7-',linewidth=linewidth,label='Bound')
plt.plot(force_ext_zero_bounds.time, (-force_ext_zero_bounds).value,'C7-',linewidth=linewidth)

plt.title('Bounded end effector external force',fontsize=fontsize1)
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

# End effector external force magnitude zero order
abs_force_ext_zero_j = DataSet()
data = abs_force_ext
jumping_indices = abs_force_ext_zero_ji
jumps = abs_force_ext_zero_j
for j in jumping_indices:
	jumps.append(data[j])

abs_force_ext_zero_bounds = DataSet()
for j in q[0]:
	abs_force_ext_zero_bounds.append(DataPoint(j.time, abs_force_ext_zero_bound))
		
fig = plt.figure(figsize=figsize, dpi=dpi)
plt.rcParams['xtick.labelsize']=fontsize2
plt.rcParams['ytick.labelsize']=fontsize2
plt.plot(abs_force_ext.time, abs_force_ext.value, 'C1-*', linewidth=linewidth, markersize=markersize1,label='Magnitude')
plt.plot(abs_force_ext_zero_j.time, abs_force_ext_zero_j.value, 'C1*', markersize=markersize2)
plt.plot(abs_force_ext_bounds.time, abs_force_ext_zero_bounds.value,'C7-',linewidth=linewidth,label='Bound')
plt.plot(abs_force_ext_bounds.time, (-abs_force_ext_zero_bounds).value,'C7-',linewidth=linewidth)

plt.title('Bounded end effector external force magnitude',fontsize=fontsize1)
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

