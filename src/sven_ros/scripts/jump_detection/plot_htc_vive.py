#!/usr/bin/python3

from readers import *
import matplotlib.pyplot as plt

bagfile = 'data/demo2.bag'
topic = '/vive/pose3'
reader = RosbagReader(bagfile, topic=topic)

poses = reader.read()
poses.align_time()
xs = []
ys = []
zs = []
ts = []

for pose_msg in poses:
	ts.append(pose_msg.time)
	xs.append(pose_msg.value.pose.position.x)
	ys.append(pose_msg.value.pose.position.y)
	zs.append(pose_msg.value.pose.position.z)
	
#topic2 = '/equilibrium_pose'
#reader2 = RosbagReader(bagfile, topic=topic2)
#poses2 = reader2.read()
#starting_time = 0
#index = 0
#for i in range(len(poses)):
#	pose = poses[i]
#	if pose.timestamp == poses2[0].timestamp:
#		index = i
#		starting_time = pose.time
#		break
#poses2.align_time(starting_time=starting_time)
#xs2 = []
#ys2 = []
#zs2 = []
#ts2 = []

#for pose_msg in poses2:
#	ts2.append(pose_msg.time / 1000000 - 0.5)
#	xs2.append(pose_msg.value.pose.position.x)
#	ys2.append(pose_msg.value.pose.position.y)
#	zs2.append(pose_msg.value.pose.position.z)
	
fontsize = 16
linewidth = 1
markersize = 3
xlim = (2.5,4.5)

plt.figure(figsize=(12, 9), dpi=80)
plt.rcParams['xtick.labelsize']=fontsize
plt.rcParams['ytick.labelsize']=fontsize

plt.plot(ts,xs,'-*C1',linewidth=linewidth, markersize=markersize)
plt.plot(ts,ys,'-*C2',linewidth=linewidth, markersize=markersize)
plt.plot(ts,zs,'-*C3',linewidth=linewidth, markersize=markersize)

plt.title('Position of the HTC Vive controller',fontsize=fontsize)
plt.xlabel('Time [s]',fontsize=fontsize)
plt.ylabel('Position [m]',fontsize=fontsize)
plt.legend(['X','Y','Z'],fontsize=fontsize)
plt.xlim(xlim)

#xs1 = []
#xs3 = []
#for i in xs:
#	xs1.append(i - xs[0])
#for i in xs2:
#	xs3.append((i - xs2[0] - xs[index]) / 0.7)
#	
#plt.figure(figsize=(16, 12), dpi=80)
#plt.rcParams['xtick.labelsize']=fontsize
#plt.rcParams['ytick.labelsize']=fontsize

#plt.plot(ts,xs1)
#plt.plot(ts2,xs3)

#plt.title('X position of the HTC Vive controller and command',fontsize=fontsize)
#plt.xlabel('Time [s]',fontsize=fontsize)
#plt.ylabel('Position [m]',fontsize=fontsize)
#plt.legend(['HTC Vive','End effector command'],fontsize=fontsize)

plt.show()
