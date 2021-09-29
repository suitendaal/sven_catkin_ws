#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

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

bound = 2.02

### Output settings
plot_external_force = True
plot_prediction_difference = True
plot_position = False
plot_velocity = True
show_figs = True
save_figs = False#True
pickle_figs = False
save_figs_location = 'figures/impact_detection_delay'
xlim = None

