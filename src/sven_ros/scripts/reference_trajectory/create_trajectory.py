#!/usr/bin/python3

import matplotlib.pyplot as plt
import config

config.end_effector.filter()
print(config.end_effector.get_time_range(1))


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
