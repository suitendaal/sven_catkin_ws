#!/usr/bin/python3

from filters import *
from trajectory import *

### Load data

# Files with demonstrations
#demos = [
#	'data/replay4.1.bag',
#	'data/replay4.2.bag',
#	'data/replay4.3.bag'
#]
demos = [
	'data/demo2.bag',
	'data/demo4.bag',
	'data/demo5.bag'
]
#demos = [
#	'data/demo10.bag',
#	'data/demo11.bag',
#	'data/demo12.bag',
#	'data/demo13.bag',
#	'data/demo14.bag',
#	'data/demo15.bag'
#]

# Impact interval indices
#impact_intervals = [
#	[(1559,)],
#	[(1573,)],
#	[(1497,)]
#]
impact_intervals = [
	[(625, 630, 634, 643)],
	[(680, 684, 688, 693, 700)],
	[(851, 859, 872)]
]
#impact_intervals = [
#	[(409, 416, 435)],
#	[(463, 469, 487)],
#	[(303, 310)],
#	[(452, 459)],
#	[(315, 322)],
#	[(449, 457, 480)]
#]
	
### Trajectory settings

## Movement primitives

rbf_width = 5e-4
n_rbfs_per_second = 35

## Filtering
position_filter = None #LeastSquaresFilter(order=2, window_length=20)
orientation_filter = None #LeastSquaresFilter(order=2, window_length=20)
velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)
rotational_velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)

## Extender
position_extender = ConstantVelocityExtender(timesteps=400, delta_time=0.005)
orientation_extender = ConstantExtender(timesteps=400, delta_time=0.005)
impact_detection_delay = 0.03# 0.1 # TODO: to be calculated based on demonstrations
impact_phase_duration = 0.25# 0.06

### Create reference output settings

## Figure settings
xlim = None
xlim = [(3,4)]
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
output_file = 'output/promps.json'

