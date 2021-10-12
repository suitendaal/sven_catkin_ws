#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo1.bag',
	'data/demo2.bag',
	'data/demo4.bag',
	'data/demo10.bag',
	'data/demo11.bag',
	'data/demo12.bag',
	'data/demo13.bag',
	'data/demo14.bag',
	'data/demo15.bag',
	'data/demo18.bag',
	'data/demo19.bag',
	'data/demo21.bag'
]

impact_intervals = [
	[(798,834,)],
	[(1190,1246,)],
	[(945,967,)],
	[(1054,1087,)],
	[(1079,1113,)],
	[(1018,1047,)],
	[(891,919,)],
	[(1144,1175,)],
	[(642,676,)],
	[(865,899,)],
	[(927,951,)],
	[(655,681,)]
]

impact_detection_delays = [
	[4,4],
	[3,4],
	[2,4],
	[5,3],
	[2,3],
	[2,4],
	[2,9],
	[3,3],
	[4,3],
	[2,3],
	[3,6],
	[3,4]
]

impact_duration = 0.04
	
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
position_extender = ConstantVelocityExtender(timesteps=0.2*500, delta_time=1/500)
orientation_extender = ConstantExtender(timesteps=0.2*500, delta_time=1/500)

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
output_file = 'output/promps.json'

