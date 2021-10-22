#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo3.3.bag',
	'data/demo4.3.bag',
	'data/demo5.3.bag',
	'data/demo6.3.bag',
	'data/demo8.3.bag',
	'data/demo9.3.bag'
]

impact_intervals = [
	[(2773,2829,)],
	[(3233,3280,)],
	[(4958,5009,)],
	[(3329,3379,)],
	[(3336,3386,)],
	[(3783,3835,)]
]


impact_detection_delays = [
	[7,6],
	[3,11],
	[9,16],
	[5,4],
	[3,4],
	[3,8]
]

impact_duration = 0.18
	
### Trajectory settings

## Movement primitives

rbf_width = 5e-4
rbfs_per_second = 35

## Filtering
position_filter = None #LeastSquaresFilter(order=2, window_length=20)
orientation_filter = None #LeastSquaresFilter(order=2, window_length=20)
velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)
rotational_velocity_filter = None #LeastSquaresFilter(order=2, window_length=20)

## Extender
position_extender = Extender(1000, impact_duration, 0.06) #ConstantVelocityExtender(timesteps=0.2*500, delta_time=1/500)
orientation_extender = Extender(1000, impact_duration, 0.06) #ConstantExtender(timesteps=0.2*500, delta_time=1/500)

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

### Evaluation settings

## ViaPoints

via_points = [
	DataSet([
	]),
	DataSet([
	]),
	DataSet([
	]),
	DataSet([
	]),
	DataSet([
	]),
	DataSet([
	])
]

### Output settings

step_size = 1/500
plot_figs = True

# Output filename
write_evaluation = True
output_file = 'output/evaluated_promps.json'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']


