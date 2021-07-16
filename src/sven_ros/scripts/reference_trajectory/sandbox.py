#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt

bagfile = 'data/replay7.2.bag'
topic = '/franka_state_controller/franka_states'
reader = RosbagReader(bagfile, topic=topic)

franka_states = reader.read()
franka_states.align_time(starting_time=0)
ts = []
qs = []
diffs = []
thetas = []
tau_Js = []
tau_J_ds = []
tau_J_diffs = []

for i in range(7):
	qs.append([])
	thetas.append([])
	diffs.append([])
	tau_Js.append([])
	tau_J_ds.append([])
	tau_J_diffs.append([])

for state in franka_states:
	ts.append(state.time / 1000000)
	for i in range(7):
		qs[i].append(state.value.q[i])
		thetas[i].append(state.value.tau_ext_hat_filtered[i])
		diffs[i].append(state.value.q[i] - state.value.theta[i])
		tau_Js[i].append(state.value.tau_J[i])
		tau_J_ds[i].append(state.value.tau_J_d[i])
		tau_J_diffs[i].append(state.value.tau_J[i] - state.value.tau_J_d[i])
	
fontsize = 16

for i in range(7):
	plt.figure(figsize=(16, 12), dpi=80)
	plt.rcParams['xtick.labelsize']=fontsize
	plt.rcParams['ytick.labelsize']=fontsize

	plt.plot(ts,tau_J_diffs[i])
	

	plt.title('Joint ' + str(i+1),fontsize=fontsize)
	plt.xlabel('Time [s]',fontsize=fontsize)
	plt.ylabel('Position [rad]',fontsize=fontsize)
	plt.legend(['Joint','Motor'],fontsize=fontsize)

plt.show()
