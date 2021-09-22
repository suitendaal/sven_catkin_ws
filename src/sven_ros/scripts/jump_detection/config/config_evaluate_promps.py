#!/usr/bin/python3

from filters import *
from trajectory import *
from .config_plot_figures import *

### Load data

# ProMPs
promp_file = 'output/promps.json'
	
### Trajectory settings

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

step_size = 0.02
plot_figs = True
xlim = None

# Output filename
write_evaluation = True
output_file = 'output/evaluated_promps.json'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']

