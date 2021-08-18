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
	
x = DataSet()
y = DataSet()
z = DataSet()
x_d = DataSet()
y_d = DataSet()
z_d = DataSet()
dx = DataSet()
dy = DataSet()
dz = DataSet()
Fx = DataSet()
Fy = DataSet()
Fz = DataSet()
Fx_d = DataSet()
Fy_d = DataSet()
Fz_d = DataSet()
F_ext = DataSet()
qs = []
dqs = []
qds = []
taus = []
tauds = []
for i in range(7):
	qs.append(DataSet())
	dqs.append(DataSet())
	qds.append(DataSet())
	taus.append(DataSet())
	tauds.append(DataSet())

for i in range(len(franka_reader.msgs)):
	dp = franka_reader.next_datapoint()
	time = dp.time
	dp = dp.value
	
	x.append(DataPoint(time, dp.position[0]))
	y.append(DataPoint(time, dp.position[1]))
	z.append(DataPoint(time, dp.position[2]))
	
	x_d.append(DataPoint(time, dp.position_desired[0]))
	y_d.append(DataPoint(time, dp.position_desired[0]))
	z_d.append(DataPoint(time, dp.position_desired[0]))
	
#	jacobian = dp.robot.jacob0(q=dp.q, T=dp.robot.fkine(dp.q))
#	vel = jacobian.dot(dp.dq)
	dx.append(DataPoint(time, dp.velocity[0]))
	dy.append(DataPoint(time, dp.velocity[1]))
	dz.append(DataPoint(time, dp.velocity[2]))
	
#	force = np.linalg.pinv(jacobian.T).dot(dp.tau)
	Fx.append(DataPoint(time, dp.force_measured[0]))
	Fy.append(DataPoint(time, dp.force_measured[1]))
	Fz.append(DataPoint(time, dp.force_measured[2]))
	
#	force_d = np.linalg.pinv(jacobian.T).dot(dp.tau_desired)
	Fx_d.append(DataPoint(time, dp.force_desired[0]))
	Fy_d.append(DataPoint(time, dp.force_desired[1]))
	Fz_d.append(DataPoint(time, dp.force_desired[2]))
	
	F_ext.append(DataPoint(time, np.linalg.norm(dp.force_external)))
	
	for j in range(7):
		qs[j].append(DataPoint(time, dp.q[j]))
		dqs[j].append(DataPoint(time, dp.dq[j]))
		qds[j].append(DataPoint(time, dp.q_desired[j]))
		taus[j].append(DataPoint(time, dp.tau_measured[j]))
		tauds[j].append(DataPoint(time, dp.tau_desired[j]))
		
x.align_time()
y.align_time()
z.align_time()
x_d.align_time()
y_d.align_time()
z_d.align_time()
dx.align_time()
dy.align_time()
dz.align_time()
Fx.align_time()
Fy.align_time()
Fz.align_time()
Fx_d.align_time()
Fy_d.align_time()
Fz_d.align_time()
F_ext.align_time()
for i in range(7):
	qs[i].align_time()
	dqs[i].align_time()
	qds[i].align_time()
	taus[i].align_time()
	tauds[i].align_time()
		
vel_estimator = LeastSquaresVelocityEstimator(window_length=20, order=2)
dq_est = []
for i in range(7):
	dq_est.append(DataSet())
for i in range(7):
	vel_estimator.reset()
	for j in range(len(qs[i])):
		vel_est, coefs = vel_estimator.update(qs[i][j])
		dq_est[i].append(vel_est)
		
ja_filter = JumpAwareVelocityFilter(LeastSquaresVelocityEstimator(window_length=20, order=2), ConstantBounder(bound=0.1))
jump_indices = []
predictions = []
for i in range(7):
	jump_indices.append([])
	predictions.append(DataSet())
for i in range(7):
	ja_filter.reset()
	for j in range(len(qs[i])):
		jump_detected, info = ja_filter.update(qs[i][j],dqs[i][j])
		if jump_detected:
			jump_indices[i].append(j)
		predictions[i].append(info[0])
		
ja_filter.bounder.bound_ = 0.05
		
jump_indices_x = []
predictions_x = DataSet()
ja_filter.reset()
for j in range(len(x)):
	jump_detected, info = ja_filter.update(x[j],dx[j])
	if jump_detected:
		jump_indices_x.append(j)
	predictions_x.append(info[0])
jump_indices.append(jump_indices_x)
predictions.append(predictions_x)

jump_indices_y = []
predictions_y = DataSet()
ja_filter.reset()
for j in range(len(y)):
	jump_detected, info = ja_filter.update(y[j],dy[j])
	if jump_detected:
		jump_indices_y.append(j)
	predictions_y.append(info[0])
jump_indices.append(jump_indices_y)
predictions.append(predictions_y)

jump_indices_z = []
predictions_z = DataSet()
ja_filter.reset()
for j in range(len(z)):
	jump_detected, info = ja_filter.update(z[j],dz[j])
	if jump_detected:
		jump_indices_z.append(j)
	predictions_z.append(info[0])
jump_indices.append(jump_indices_z)
predictions.append(predictions_z)
		
print(jump_indices)
		
#ls_filter = LeastSquaresFilter(window_length=20, order=3)
#q_est = []
#for i in range(7):
#	q_est.append(DataSet())
#for i in range(7):
#	ls_filter.reset()
#	for j in range(len(qs[i])):
#		q_estimation, coefs = ls_filter.update(qs[i][j])
#		q_est[i].append(q_estimation)
		
ja_filter2 = JumpAwareFilter(LeastSquaresFilter(window_length=20, order=2), ConstantBounder(bound=0.1))
jump_indices2 = []
predictions2 = []
for i in range(7):
	jump_indices2.append([])
	predictions2.append(DataSet())
for i in range(7):
	ja_filter2.reset()
	for j in range(len(dqs[i])):
		jump_detected, info = ja_filter2.update(dqs[i][j])
		if jump_detected:
			jump_indices2[i].append(j)
		predictions2[i].append(info[0])
		
print(jump_indices2)

ja_filter3 = JumpAwareFilter(LeastSquaresFilter(window_length=20, order=0), ConstantBounder(bound=20))
jump_indices3 = []
predictions3 = DataSet()
for j in range(len(F_ext)):
	jump_detected, info = ja_filter3.update(F_ext[j])
	if jump_detected:
		jump_indices3.append(j)
	predictions3.append(info[0])

print(jump_indices3)
		
#plt.figure()
#for i in range(7):
#	plt.plot(dq_est[i].time, dq_est[i].value,'C' + str(i) + '-*',linewidth=4)
#for i in range(7):
#	plt.plot(dqs[i].time, dqs[i].value,'C' + str(i+1) + '-*',linewidth=3)
#for i in range(7):
#	plt.plot(qs[i].diff().time, qs[i].diff().value,'C' + str(i+2) + '-*',linewidth=2)

#plt.figure()
#for i in range(7):
#	plt.plot(qs[i].time, (qs[i]-qs[i][0]).value,'C' + str(i) + '-*',linewidth=2)
#	
#plt.figure()
#for i in range(7):
#	plt.plot(predictions[i].time, abs(predictions[i]-qs[i]).value,'C' + str(i) + '-*',linewidth=2)

#plt.figure()
#for i in range(7):
#	plt.plot(dqs[i].time, dqs[i].value,'C' + str(i) + '-*',linewidth=2)
#	
#plt.figure()
#for i in range(7):
#	plt.plot(predictions[i].time, abs(predictions[i]-dqs[i]).value,'C' + str(i) + '-*',linewidth=2)
#	
#plt.figure()
#for i in range(7):
#	plt.plot(predictions2[i].time, abs(predictions2[i]-dqs[i]).value,'C' + str(i) + '-*',linewidth=2)
#	
#plt.figure()
#plt.plot(dx.time, dx.value,'C0-*',linewidth=2)
#plt.plot(dy.time, dy.value,'C1-*',linewidth=2)
#plt.plot(dz.time, dz.value,'C2-*',linewidth=2)
#	
#plt.figure()
#plt.plot(predictions_x.time, abs(predictions_x-dx).value,'C0-*',linewidth=2)
#plt.plot(predictions_y.time, abs(predictions_y-dy).value,'C1-*',linewidth=2)
#plt.plot(predictions_z.time, abs(predictions_z-dz).value,'C2-*',linewidth=2)

plt.figure()
plt.plot(F_ext.time, F_ext.value)

plt.figure()
plt.plot(predictions3.time, abs(predictions3 - F_ext).value)
	
#i = 5
#plt.figure()
#plt.plot(dqs[i].time, dqs[i].value,'C' + str(i) + '-*',linewidth=2)

#plt.figure()
#plt.plot(predictions[i].time, abs(predictions[i]-dqs[i]).value,'C' + str(i) + '-*',linewidth=2)
	
plt.show()

