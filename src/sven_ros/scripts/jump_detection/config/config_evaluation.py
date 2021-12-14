#!/usr/bin/python3

from filters import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
]

baseline = [
	'data/replay1.bag',
	'data/replay2.bag',
	'data/replay3.bag',
	'data/replay4.bag'
]

executions = [
	'data/replay5.bag',
	'data/replay6.bag',
	'data/replay7.bag',
	'data/replay8.bag',
	'data/replay9.bag',
	'data/replay10.bag',
	'data/replay11.bag',
	'data/replay12.bag',
	'data/replay13.bag',
	'data/replay14.bag',
	'data/replay15.bag',
	'data/replay16.bag',
	'data/replay17.bag',
	'data/replay18.bag',
	'data/replay19.bag',
	'data/replay20.bag',
	'data/replay21.bag',
	'data/replay22.bag',
	'data/replay23.bag',
	'data/replay24.bag',
	'data/replay25.bag',
	'data/replay26.bag',
	'data/replay27.bag',
	'data/replay28.bag'
]

### Output file
output_plot_all = 'figures_git/all_evaluations.pdf'
output_plot_all_zoom = 'figures_git/all_evaluations_zoom.pdf'

