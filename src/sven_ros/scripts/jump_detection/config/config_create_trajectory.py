#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo26.bag',
	'data/demo27.bag',
	'data/demo28.bag',
	'data/demo29.bag',
	'data/demo30.bag',
	'data/demo32.bag',
	'data/demo34.bag',
	'data/demo35.bag',
	'data/demo36.bag',
	'data/demo37.bag',
	'data/demo39.bag',
	'data/demo41.bag'
]

impact_intervals = [
	[(1582,)],
	[(1489,)],
	[(1545,)],
	[(1642,)],
	[(1824,1879,)],
	[(1691,)],
	[(1641,)],
	[(1683,)],
	[(1743,1769,)],
	[(1805,1854,)],
	[(1816,)],
	[(1699,)]
]

impact_detection_delays = [
	[6],
	[9],
	[4],
	[6],
	[4,7],
	[5],
	[3],
	[5],
	[3,4],
	[5,5],
	[7],
	[8]
]

#demos = [
#	'data/demo2.bag',
#	'data/demo5.bag',
#	'data/demo8.bag',
#	'data/demo9.bag',
#	'data/demo10.bag',
#	'data/demo15.bag'
#]

#impact_intervals = [
#	[(1213,1261,)],
#	[(1218,1271,)],
#	[(996,1062,)],
#	[(1305,1367,)],
#	[(1034,1084,1136,)],
#	[(949,1016,)]
#]

#impact_detection_delays = [
#	[8,0],
#	[2,4],
#	[8,4],
#	[2,3],
#	[6,1,4],
#	[7,2]
#]

impact_duration = 0.2
	
### Trajectory settings

## Movement primitives

rbf_width = 5e-4 / 2
rbfs_per_second = 35 * 2

## Filtering
position_filter = None #LeastSquaresFilter(order=2, window_length=20)
orientation_filter = None #LeastSquaresFilter(order=2, window_length=20)
velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)
rotational_velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)

## Extender
frequency, timespan = 500, 2.0
position_extender = ConstantVelocityExtender(frequency, impact_duration, timespan) #ConstantVelocityExtender(timesteps=0.2*500, delta_time=1/500)
orientation_extender = Extender(frequency, impact_duration, timespan)
force_extender = ConstantForceExtender3(frequency, impact_duration, timespan)
torque_extender = ConstantForceExtender3(frequency, impact_duration, timespan)

### Create reference output settings

## Figure settings
xlim = [-0.05, 0.05]
#xlim = [(3,4)]
#figsize = (16, 12)
#dpi = 80
#linewidth = 1
#markersize1 = 10
#markersize2 = 3
#fontsize1 = 20
#fontsize2 = 16
## Full trajectory
#xlim = [None]
## Zoomed in on first impact
##xlim = [(8.6, 8.8)]
## Zoomed in on impact interval
##xlim = [(8.6, 11)]
## Zoomed in on high force
##xlim = [(8.5, 11)]
#labels = ['X','Y','Z']

## Plotting settings
#plot_pos_mp = True
#plot_pos_data = True
#plot_vel_mp = True
#plot_vel_data = True
#save_trajectory_figs = True
#save_trajectory_figs_location = 'figures/create_trajectory'
#show_trajectory_figs = False

# Output filename
write_mps = True
promps_output_file = 'output/promps.json'

### Evaluation settings

## ViaPoints

via_points = [
	DataSet([
	]), # X
	DataSet([
	]), # Y
	DataSet([
	]), # Z
	DataSet([
	]), # phi
	DataSet([
	]), # theta
	DataSet([
	]), # psi
	DataSet([
	]), # Fx
	DataSet([
	]), # Fy
	DataSet([
	]), # Fz
	DataSet([
	]), # Tphi
	DataSet([
	]), # Ttheta
	DataSet([
	]) # Tpsi
]

### Output settings

step_size = 1/500
plot_figs = True

# Output filename
write_evaluation = True
evaluated_promps_output_file = 'output/evaluated_promps.csv'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']


