#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt
from datalib import *
from filters import *

bagfile = 'data/replay1.1.bag'
#topic = '/franka_state_controller/franka_states'
franka_reader = FrankaStateReader(bagfile)
pose_reader = CartesianPoseReader(bagfile)
joint_readers = []
for i in range(7):
	joint_readers.append(JointReader(bagfile, joint=i+1))
	
x = DataSet(timefactor = 1000000)
y = DataSet(timefactor = 1000000)
z = DataSet(timefactor = 1000000)
x_d = DataSet(timefactor = 1000000)
y_d = DataSet(timefactor = 1000000)
z_d = DataSet(timefactor = 1000000)
dx = DataSet(timefactor = 1000000)
dy = DataSet(timefactor = 1000000)
dz = DataSet(timefactor = 1000000)
Fx = DataSet(timefactor = 1000000)
Fy = DataSet(timefactor = 1000000)
Fz = DataSet(timefactor = 1000000)
Fx_d = DataSet(timefactor = 1000000)
Fy_d = DataSet(timefactor = 1000000)
Fz_d = DataSet(timefactor = 1000000)
qs = []
dqs = []
qds = []
taus = []
tauds = []
tau_ext = []
F_ext = DataSet(timefactor = 1000000)

for i in range(7):
	qs.append(DataSet(timefactor=1000000))
	dqs.append(DataSet(timefactor=1000000))
	qds.append(DataSet(timefactor=1000000))
	taus.append(DataSet(timefactor=1000000))
	tauds.append(DataSet(timefactor=1000000))
	tau_ext.append(DataSet(timefactor=1000000))

for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	timestamp = dp.timestamp
	time = dp.time
	dp = dp.value
	
	x.append(DataPoint(timestamp, dp.x, time=time))
	y.append(DataPoint(timestamp, dp.y, time=time))
	z.append(DataPoint(timestamp, dp.z, time=time))
	
	x_d.append(DataPoint(timestamp, dp.x_desired, time=time))
	y_d.append(DataPoint(timestamp, dp.y_desired, time=time))
	z_d.append(DataPoint(timestamp, dp.z_desired, time=time))
	
	jacobian = dp.robot.jacob0(q=dp.q, T=dp.robot.fkine(dp.q))
	vel = jacobian.dot(dp.dq)
	dx.append(DataPoint(timestamp, vel[0], time=time))
	dy.append(DataPoint(timestamp, vel[1], time=time))
	dz.append(DataPoint(timestamp, vel[2], time=time))
	
	force = np.linalg.pinv(jacobian.T).dot(dp.tau)
	Fx.append(DataPoint(timestamp, force[0], time=time))
	Fy.append(DataPoint(timestamp, force[1], time=time))
	Fz.append(DataPoint(timestamp, force[2], time=time))
	
	force_d = np.linalg.pinv(jacobian.T).dot(dp.tau_desired)
	Fx_d.append(DataPoint(timestamp, force_d[0], time=time))
	Fy_d.append(DataPoint(timestamp, force_d[1], time=time))
	Fz_d.append(DataPoint(timestamp, force_d[2], time=time))
	
	force_ext = np.linalg.pinv(jacobian.T).dot(dp.external_torque)
	F_ext.append(DataPoint(timestamp, np.linalg.norm(force_ext[0:3]), time=time))
	
	for j in range(7):
		qs[j].append(DataPoint(timestamp, dp.q[j], time=time))
		dqs[j].append(DataPoint(timestamp, dp.dq[j], time=time))
		qds[j].append(DataPoint(timestamp, dp.q_desired[j], time=time))
		taus[j].append(DataPoint(timestamp, dp.tau[j], time=time))
		tauds[j].append(DataPoint(timestamp, dp.tau_desired[j], time=time))
		tau_ext[j].append(DataPoint(timestamp, dp.external_torque[j], time=time))
		
## End effector position
#plt.figure()
##plt.plot(x.time(),(x-x[0]).values())
##plt.plot(y.time(),(y-y[0]).values())
#plt.plot(z.time(),(z-z[0]).values())

### End effector velocity
##x1,y1 = z.diff().get_xy()
#x2,y2 = dz.get_xy()
#plt.figure()
##plt.plot(x1,y1)
#plt.plot(x2,y2,'-*')

### Joint positions
##plt.figure()
##for i in range(7):
##	plt.plot(qs[i].time(),(qs[i] - qs[i][0]).values())
##	
### Joint velocities
##plt.figure()
##for i in range(7):
##	plt.plot(dqs[i].time(),dqs[i].values())
#	
## Joint torque
##plt.figure()
##plt.plot(taus[4].time(),(taus[4]-tauds[4]).values())

## End effector force
#plt.figure()
#plt.plot(Fx.time(), Fx.values())
#plt.plot(Fx.time(), Fx_d.values())

##plt.show()

### Jump aware filter

## Default jump detector
#diff_torque = dz
#filter = LeastSquaresFilter(window_length=200, order=4)
#vel_est = LeastSquaresVelocityEstimator(window_length=20, order=3)
#predictor = BasePredictor(order=4)
#bounder = BaseBounder(bound=0.3)
#ja_filter = JumpAwareFilter(filter, vel_est, predictor, bounder, max_window_length=50)

#filtered_data, vel_data, jumping_indexes, info = ja_filter.filter(diff_torque)
#predictions = info[0]
#bounds = info[1]
#plt.figure()
#plt.plot(diff_torque.time(), abs(diff_torque - predictions).values(),'-*')
#plt.plot(bounds.time(), bounds.values())

##plt.figure()
##plt.plot(Fx.time(), (filtered_data - Fx_d).values())
##plt.plot(Fx.time(), Fx_d.values())

plt.figure()
for i in range(7):
	plt.plot(tau_ext[i].time(), tau_ext[i].values())
	
plt.figure()
plt.plot(F_ext.time(), F_ext.values())

plt.show()



