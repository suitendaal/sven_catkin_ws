#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import config
from config import end_effector

end_effector.filter()
plt.figure()
for phase in range(config.n_phases):

	# Create promps
	rbfs = end_effector.create_basis_functions(phase=phase, width=config.rbf_width, rbfs_per_second=config.n_rbfs_per_second)
	end_effector.create_promps(phase=phase, rbfs=rbfs, promp_type='all')
	end_effector.align_promp_time(phase)
	
	# Evaluate promps
	x = np.arange(end_effector.pos_promps[phase][2].get_starting_time(), config.end_effector.pos_promps[phase][2].get_ending_time(), 0.01).tolist()
	mu,sigma = end_effector.pos_promps[phase][2].evaluate(x, derivative=0)
	
	plt.plot(x,mu)
	
z1 = end_effector.cartesian_data[0].get_z_filtered()
z1.align_time(z1[end_effector.cartesian_data[0].jump_intervals[0][0]-1].time - z1[0].time)
x1,y1 = z1.get_xy()
for i in range(len(x1)):
	x1[i] = x1[i] + end_effector.pos_promps[phase][2].starting_time + 1
plt.plot(x1,y1)

plt.show()

#phase = 0
#config.end_effector.filter()
#rbfs = config.end_effector.create_basis_functions(phase=phase, width=config.rbf_width, rbfs_per_second=config.n_rbfs_per_second)

#print(len(rbfs), " RBFs")
#t_start,t_end = config.end_effector.get_time_range(phase)
#print(t_start,t_end)

#starting_time = 0

#config.end_effector.create_promps(phase=phase, rbfs=rbfs, promp_type='all')
#end_effector.align_promp_time(phase)
#x = np.arange(config.end_effector.pos_promps[phase][2].get_starting_time(), config.end_effector.pos_promps[phase][2].get_ending_time(), 0.01).tolist()
#y = []
#for i in x:
#	y.append(i + starting_time)


##config.end_effector.pos_promps[phase][2].align_time(starting_time)
#mu,sigma = config.end_effector.pos_promps[phase][2].evaluate(y, derivative=0)
#print(config.end_effector.pos_promps[phase][2].starting_time, config.end_effector.pos_promps[phase][2].ending_time)
##print(config.end_effector.pos_promps[0].toJSON())

#plt.figure()
#plt.plot(y,mu)

#z1 = config.end_effector.cartesian_data[0].get_z_filtered(phase)
#config.end_effector.align_time(z1,phase)
#x1,y1 = z1.get_xy()
#plt.plot(x1,y1)

#z2 = config.end_effector.cartesian_data[1].get_z_filtered(phase)
#config.end_effector.align_time(z2,phase)
#x2,y2 = z2.get_xy()
#plt.plot(x2,y2)

#z3 = config.end_effector.cartesian_data[2].get_z_filtered(phase)
#config.end_effector.align_time(z3,phase)
#x3,y3 = z3.get_xy()
#plt.plot(x3,y3)

#plt.legend(['mu','z1','z2','z3'])


##plt.figure()
##x0,y0 = config.end_effector.cartesian_data[0].y.get_xy()
##x1,y1 = config.end_effector.cartesian_data[0].y_filtered.get_xy()
##plt.plot(x0,y0)
##plt.plot(x1,y1)

##plt.figure()
##x2,y2 = config.end_effector.cartesian_data[0].x_diff.get_xy()
##x3,y3 = config.end_effector.cartesian_data[0].x_vel_est.get_xy()
##plt.plot(x2,y2)
##plt.plot(x3,y3)

##plt.figure()
##x4,y4 = config.end_effector.cartesian_data[0].q[0].get_xy()
##x5,y5 = config.end_effector.cartesian_data[0].q_filtered[0].get_xy()
##plt.plot(x4,y4)
##plt.plot(x5,y5)

#plt.show()
