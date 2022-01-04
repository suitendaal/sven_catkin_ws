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
	'data/replay28.bag',
	'data/replay29.bag',
	'data/replay30.bag',
	'data/replay31.bag',
	'data/replay32.bag',
	'data/replay33.bag',
	'data/replay34.bag',
	'data/replay35.bag',
	'data/replay36.bag',
	'data/replay37.bag',
	'data/replay38.bag',
	'data/replay39.bag',
	'data/replay40.bag',
	'data/replay41.bag'
]

titles = [
	'data/replay28.bag',
	'data/replay29.bag',
	'data/replay30.bag',
	'data/replay31.bag',
	'data/replay32.bag',
	'data/replay33.bag',
	'data/replay34.bag',
	'data/replay35.bag',
	'data/replay36.bag',
	'data/replay37.bag',
	'data/replay38.bag',
	'data/replay39.bag',
	'data/replay40.bag',
	'data/replay41.bag'
]

### Output file
output_plot_all = 'figures_git/all_evaluations.pdf'
output_plot_all_zoom = 'figures_git/all_evaluations_zoom.pdf'

