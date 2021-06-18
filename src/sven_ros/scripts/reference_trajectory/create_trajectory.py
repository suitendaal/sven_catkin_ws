#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import config

phase = 0
config.end_effector.filter()
rbfs = config.end_effector.create_basis_functions(phase=phase, width=config.rbf_width, rbfs_per_second=config.n_rbfs_per_second)
print(len(rbfs), " RBFs")
t_start,t_end = config.end_effector.get_time_range(phase)

x = np.arange(t_start, t_end, 0.01).tolist()
#for j in range(len(rbfs)):
#	y = []
#	for i in x:
#		y.append(rbfs[j].evaluate(i))
#	plt.plot(x,y)
#plt.show()

config.end_effector.create_promps(phase=phase, rbfs=rbfs, promp_type='all')
mu,sigma = config.end_effector.pos_promps[2].evaluate(x)
print(t_start,t_end)

plt.figure()
plt.plot(x,mu)
z1 = config.end_effector.cartesian_data[0].get_z(phase)
z2 = config.end_effector.cartesian_data[1].get_z(phase)
z3 = config.end_effector.cartesian_data[2].get_z(phase)
config.end_effector.align_time([z1, z2, z3],phase)
x1,y1 = z1.get_xy()
x2,y2 = z2.get_xy()
x3,y3 = z3.get_xy()
plt.plot(x1,y1)
plt.plot(x2,y2)
plt.plot(x3,y3)
plt.show()


#plt.figure()
#x0,y0 = config.end_effector.cartesian_data[0].x.get_xy()
#x1,y1 = config.end_effector.cartesian_data[0].x_filtered.get_xy()
#plt.plot(x0,y0)
#plt.plot(x1,y1)

#plt.figure()
#x2,y2 = config.end_effector.cartesian_data[0].x_diff.get_xy()
#x3,y3 = config.end_effector.cartesian_data[0].x_vel_est.get_xy()
#plt.plot(x2,y2)
#plt.plot(x3,y3)

#plt.figure()
#x4,y4 = config.end_effector.cartesian_data[0].q[0].get_xy()
#x5,y5 = config.end_effector.cartesian_data[0].q_filtered[0].get_xy()
#plt.plot(x4,y4)
#plt.plot(x5,y5)

#plt.show()
