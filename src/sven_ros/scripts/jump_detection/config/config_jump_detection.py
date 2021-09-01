#!/usr/bin/python3

from filters import *

### Load data

# Files with demonstrations
demos = [
	'data/replay4.1.bag',
	'data/replay4.2.bag',
	'data/replay4.3.bag'
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
save_figs = False
pickle_figs = False
save_figs_location = 'figures/detect_jumps'

# Figure settings
figsize = (16, 12)
dpi = 80
linewidth = 1
markersize1 = 10
markersize2 = 3
fontsize1 = 20
fontsize2 = 16
# Full trajectory
xlim = [
	None,
	None,
	None
]
# Zoomed in on first impact
#xlim = [(8.6, 8.8)]
# Zoomed in on impact interval
#xlim = [(8.6, 11)]
# Zoomed in on high force
#xlim = [(8.5, 11)]
labels = ['X','Y','Z']

