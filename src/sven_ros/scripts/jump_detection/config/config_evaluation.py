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
	'data/replay_test.bag'
]

titles = [
	'Experiment A',
#	'Experiment C',
#	'Experiment B',
#	'Experiment D',
#	'Experiment I',
#	'Experiment K',
#	'Experiment J',
#	'Experiment L',
#	'Experiment Q',
#	'Experiment S',
#	'Experiment R',
#	'Experiment T',
#	'Experiment E',
#	'Experiment G',
#	'Experiment F',
#	'Experiment H',
#	'Experiment M',
#	'Experiment N',
#	'Experiment U',
#	'Experiment W'
]

### Output file
output_plot_all = 'figures_git/all_evaluations.pdf'
output_plot_all_zoom = 'figures_git/all_evaluations_zoom.pdf'

