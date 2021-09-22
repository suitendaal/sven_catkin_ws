#!/usr/bin/python3

from filters import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
#demos = [
#	'data/replay4.1.bag',
#	'data/replay4.2.bag',
#	'data/replay4.3.bag'
#]
#demos = [
#	'data/demo2.bag',
#	'data/demo4.bag',
#	'data/demo5.bag'
#]
demos = [
	'data/demo10.bag',
	'data/demo11.bag',
	'data/demo12.bag',
	'data/demo13.bag',
	'data/demo14.bag',
	'data/demo15.bag'
]

### Jump detector
predictor = LeastSquaresFilter(order=2)
bounder = ConstantBounder(bound=6)
jump_detector = JumpAwareFilter(predictor, bounder, max_window_length=20)
	
### Detect Jumps output settings

# Output settings
plot_external_force = True
plot_prediction_difference = True
plot_position = True
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
xlim = None
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

