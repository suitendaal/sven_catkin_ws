#!/usr/bin/python3

from filters import *
from trajectory import *

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
		ViaPoint(0, 0.16),
		ViaPoint(0, 0, derivative=1),
		ViaPoint(9, 0.075)
	]),
	DataSet([
	]),
	DataSet([
	]),
	DataSet([
	])
]

### Output settings

step_size = 0.1
plot_figs = True

# Output filename
write_evaluation = True
output_file = 'output/evaluated_promps.json'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']

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

