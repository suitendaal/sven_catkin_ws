#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

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

