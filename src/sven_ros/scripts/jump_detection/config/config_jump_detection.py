#!/usr/bin/python3

from filters import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
#demos = [
#	'data/demo4.bag',
#	'data/demo5.bag',
#	'data/demo6.bag',
#	'data/demo9.bag',
#	'data/demo10.bag'
#]

demos = [
	'data/demo13.bag',
	'data/demo14.bag',
	'data/demo15.bag',
	'data/demo16.bag',
	'data/demo17.bag'
]

#demos = [
#	'data/demo20.bag',
#	'data/demo21.bag',
#	'data/demo22.bag',
#	'data/demo23.bag',
#	'data/demo24.bag'
#]

#demos = [
#	'data/demo25.bag',
#	'data/demo26.bag',
#	'data/demo27.bag',
#	'data/demo28.bag',
#	'data/demo29.bag'
#]

#demos = [
#	'data/demo33.bag',
#	'data/demo34.bag',
#	'data/demo35.bag',
#	'data/demo36.bag',
#	'data/demo37.bag',
#	'data/demo38.bag'
#]

### Jump detector
predictor = LeastSquaresFilter(order=1)
bounder = ConstantBounder(bound=None)#4.7487593326350535)
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

