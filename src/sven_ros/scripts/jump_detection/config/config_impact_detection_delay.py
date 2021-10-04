#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo1.bag',
	'data/demo4.bag',
	'data/demo5.bag',
	'data/demo10.bag',
	'data/demo11.bag',
	'data/demo12.bag',
	'data/demo13.bag',
	'data/demo14.bag',
	'data/demo15.bag',
	'data/demo19.bag',
	'data/demo21.bag'
]

# Impact interval indices
impact_intervals = [
	[(798, 835)],
	[(946, 968)],
	[(1524, 1557)],
	[(1055, 1089)],
	[(1079, 1115)],
	[(1019, 1051)],
	[(891, 925)],
	[(1144, 1177)],
	[(643, 683)],
	[(927, 958)],
	[(655, 685)]
]
jump_intervals = [
	[(798, 808, 835, 860)],
	[(946, 955, 968)],
	[(1524, 1534, 1557)],
	[(1055, 1066, 1089)],
	[(1079, 1090, 1115)],
	[(1019, 1029, 1051)],
	[(891, 900, 925)],
	[(1144, 1156, 1177, 1208)],
	[(643, 654, 683, 701)],
	[(927, 937, 958)],
	[(655, 663, 685)]
]

bound = [2.16, 2.16]

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

