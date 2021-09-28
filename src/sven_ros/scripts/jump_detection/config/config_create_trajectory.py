#!/usr/bin/python3

from filters import *
from trajectory import *

### Load data

# Files with demonstrations
demos = [
	'data/demo3.bag',
	'data/demo5.bag',
	'data/demo6.bag',
	'data/demo8.bag',
	'data/demo9.bag',
	'data/demo10.bag',
	'data/demo11.bag',
	'data/demo13.bag',
	'data/demo14.bag',
	'data/demo15.bag'
]

# Impact interval indices
impact_intervals = [
	[(411, 431)],
	[(493, 511)],
	[(575, 589)],
	[(416, 438)],
	[(432, 454)],
	[(409, 426)],
	[(463, 476)],
	[(452, 468)],
	[(315, 333)],
	[(449, 466)]
]
	
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
impact_detection_delay = 0.012
impact_phase_duration = 0.25# 0.06

### Create reference output settings

## Figure settings
xlim = None
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
output_file = 'output/promps.json'

