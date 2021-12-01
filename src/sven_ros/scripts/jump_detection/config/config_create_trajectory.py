#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo10.bag',
	'data/demo11.bag',
	'data/demo12.bag'
]

impact_intervals = [
	[(1702,1851,)],
	[(1740,1866,)],
	[(1609,1789,)]
]


impact_detection_delays = [
	[5,4],
	[5,7],
	[3,4]
]

impact_duration = 0.18
	
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
frequency, timespan = 500, 1.0
position_extender = ConstantVelocityExtender(frequency, impact_duration, timespan) #ConstantVelocityExtender(timesteps=0.2*500, delta_time=1/500)
orientation_extender = Extender(frequency, impact_duration, timespan)
force_extender = Extender(frequency, impact_duration, timespan)
torque_extender = Extender(frequency, impact_duration, timespan)

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


