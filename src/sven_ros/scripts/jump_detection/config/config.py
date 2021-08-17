#!/usr/bin/python3

from filters import *
from readers import *
from models import *

# Joints
n_joints = 7

# Impacts
n_phases = 2

### Load data

# Files with demonstrations
demos = [
	'data/replay6.2.bag',
	'data/replay7.2.bag'
]

# Jump intervals
jump_intervals = [
	[(538,560)],
	[(609,627)]
]

# Load data files
joints = []
for i in range(n_joints):
	joints.append(Joint(i+1))
end_effector = EndEffector(n_phases)

for i in range(len(demos)):
	filename = demos[i]

	# Read joint data
	for j in range(n_joints):
		joints[j].append_data(get_joint_data(filename,j+1)[0])
		
	# Read cartesian data
	x, y, z, q = get_cartesian_data(filename)
	jump_intervals_set = jump_intervals[i]
	end_effector.append_data(x, y, z, q, jump_intervals_set)
	
### Movement primitives

rbf_width = 5e-4
n_rbfs_per_second = 35

### Jump detectors

# Default jump detector
d_filter = LeastSquaresFilter(window_length=20, order=3)
d_vel_est = LeastSquaresVelocityEstimator(window_length=20, order=3)
#d_pred = WeightedPredictor(order=3,frequency=3)
d_pred = BasePredictor(order=3)
d_bounder = BaseBounder(bound=0.01)
default_jump_detector = JumpAwareFilter(d_filter, d_vel_est, d_pred, d_bounder, max_window_length=20)

# Jump detector per joint
for i in range(n_joints):
	joints[i].jump_detector = default_jump_detector.copy()
#joints[0].jump_detector.bounder.bound = 0.014
#joints[3].jump_detector.bounder.bound = 0.0055
#joints[4].jump_detector.bounder.bound = 0.008
#joints[5].jump_detector.bounder.bound = 0.009
#joints[6].jump_detector.bounder.bound = 0.007
	
### Detect Jumps output settings

plot_pos = True
plot_pred = True
plot_vel = True
filter_cartesian_with_jump = False
plot_cartesian_pos = False
plot_cartesian_vel = False
show_jumping_indexes = True
save_figs = True
pickle_figs = True
save_figs_location = 'figures/detect_jumps'
show_figs = False
#jumps_x_lim = [(3.6,4.2),(4.3,4.9),(4.6,5.2)]
jumps_x_lim = [(0,5),(0,5)]

### Filtering settings

end_effector.position_filter = LeastSquaresFilter(window_length=20, order=3)
end_effector.velocity_estimator = LeastSquaresVelocityEstimator(window_length=20, order=3)
end_effector.orientation_filter = LeastSquaresFilter(window_length=20, order=3)
end_effector.position_extender = ConstantVelocityExtender(timesteps=100, delta_time=0.01)

### Create reference output settings

# Plotting settings
plot_pos_mp = True
plot_pos_data = True
plot_vel_mp = True
plot_vel_data = True
save_trajectory_figs = True
save_trajectory_figs_location = 'figures/create_trajectory'
show_trajectory_figs = False

# Output filename
write_mps = True
output_file = 'output/data.json'

