#!/usr/bin/python3

from filters import *
from readers import *
from models import *

# Joints
n_joints = 7

# Impacts
n_phases = 1

### Load data

# Files with demonstrations
demos = [
	'data/traj3.1_5b.bag',
	'data/traj4.1_5b.bag',
	'data/traj6.1_5.bag'
]

# Jump intervals
jump_intervals = [
	[(353,383)],
	[(404,466)],
	[(495, 501)]
]

# Load data files
joints = []
for i in range(n_joints):
	joints.append(Joint(i+1))
end_effector = EndEffector()

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
d_pred = WeightedPredictor(order=3,frequency=3)
d_bounder = BaseBounder(bound=0.02)
default_jump_detector = JumpAwareFilter(d_filter, d_vel_est, d_pred, d_bounder, max_window_length=20)

# Jump detector per joint
for i in range(n_joints):
	joints[i].jump_detector = default_jump_detector.copy()
	
### Detect Jumps settings

plot_pos = False
plot_pred = False
plot_vel = False
show_jumping_indexes = True

### Filtering settings
end_effector.position_filter = LeastSquaresFilter(window_length=20, order=3)
end_effector.velocity_estimator = LeastSquaresVelocityEstimator(window_length=20, order=3)
end_effector.orientation_filter = LeastSquaresFilter(window_length=20, order=3)

