#!/usr/bin/python3

from filters import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
#demos = [
#	'data/demo3.3.bag',
#	'data/demo4.3.bag',
#	'data/demo5.3.bag',
#	'data/demo6.3.bag',
#	'data/demo8.3.bag',
#	'data/demo9.3.bag'
#]
demos = [
	'data/demo3.2.bag'
]

### Jump detector
predictor = LeastSquaresFilter(order=1)
bounder = ConstantBounder(bound=5)
jump_detector = JumpAwareExternalForceFilter(predictor, bounder, max_window_length=24)
	
### Detect Jumps output settings

# Output settings
plot_external_force = True
plot_prediction_difference = True
plot_position = False
plot_velocity = True
show_figs = True
save_figs = False#True
pickle_figs = False
save_figs_location = 'figures/detect_jumps'

## Figure settings
#figsize = (16, 12)
#dpi = 80
#linewidth = 1
#markersize1 = 10
#markersize2 = 3
#fontsize1 = 20
#fontsize2 = 16
## Full trajectory
xlim = [-0.05, 0.05]
#xlim = [
#	xlim[0],
#	None,
#	None
#]
## Zoomed in on first impact
##xlim = [(8.6, 8.8)]
## Zoomed in on impact interval
##xlim = [(8.6, 11)]
## Zoomed in on high force
##xlim = [(8.5, 11)]
##xlim = [(2.6,4.5)]
#labels = ['X','Y','Z']

