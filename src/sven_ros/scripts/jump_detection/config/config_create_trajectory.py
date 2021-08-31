#!/usr/bin/python3

from filters import *
from trajectory import *

### Load data

# Files with demonstrations
demos = [
	'data/replay4.1.bag'
]

# Impact interval indices
impact_intervals = [
	[(1559,)]
]
	
### Trajectory settings

## Movement primitives

rbf_width = 5e-4
n_rbfs_per_second = 35

## Filtering
position_filter = LeastSquaresFilter(order=2, window_length=20)
orientation_filter = LeastSquaresFilter(order=2, window_length=20)
velocity_filter = LeastSquaresFilter(order=2, window_length=20)
rotational_velocity_filter = LeastSquaresFilter(order=2, window_length=20)

## Extender
position_extender = ConstantVelocityExtender(timesteps=100, delta_time=0.01)
orientation_extender = ConstantVelocityExtender(timesteps=100, delta_time=0.01)

### Create reference output settings

# Figure settings
figsize = (16, 12)
dpi = 80
linewidth = 1
markersize1 = 10
markersize2 = 3
fontsize1 = 20
fontsize2 = 16
# Full trajectory
xlim = [None]
# Zoomed in on first impact
#xlim = [(8.6, 8.8)]
# Zoomed in on impact interval
#xlim = [(8.6, 11)]
# Zoomed in on high force
#xlim = [(8.5, 11)]
labels = ['X','Y','Z']

# Plotting settings
plot_pos_mp = True
plot_pos_data = True
plot_vel_mp = True
plot_vel_data = True
save_trajectory_figs = True
save_trajectory_figs_location = 'figures/create_trajectory'
show_trajectory_figs = False

# Output filename
write_mps = True
output_file = 'output/data.json'

