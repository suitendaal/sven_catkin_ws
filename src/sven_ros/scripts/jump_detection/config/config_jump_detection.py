#!/usr/bin/python3

from filters import *
from .config_plot_figures import *

### Load data

# Files with demonstrations
demos = [
	'data/demo8.bag',
	'data/demo9.bag',
	'data/demo10.bag'
]

### Jump detector
#predictor = LeastSquaresFilter(order=1)
#tmp = 1.12#0.55#1.12
#bound = [tmp/2 / 0.001**2, 0, tmp/2]#[tmp/2 / 0.001**3, 0, 0, tmp/2] #[tmp / 0.001**2, 0, 0] #[tmp/2 / 0.001**2, 0, tmp/2] #[tmp/3 / 0.001**2, tmp/3 / 0.001, tmp/3] #[500000, 0, 0]
#bounder = NumericalBounder(bound=bound)# ConstantBounder(bound=0.5)
#jump_detector = JumpAwareExternalForceFilter(predictor, bounder, max_window_length=14)
predictor = ForceDerivativePredictor(order=1)
bounder = ForceDerivativeBounder(bound=620)
jump_detector = JumpAwareExternalForceFilter(predictor, bounder, max_window_length=1)
	
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

