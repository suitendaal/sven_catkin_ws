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
	'data/traj2.1_5.bag',
	'data/traj3.1_5b.bag',
	'data/traj4.1_5b.bag',
	'data/traj6.1_5.bag'
]

# Load data files
joints = []
for i in range(n_joints):
	joints.append(Joint(i+1))
cartesian_data = []

for filename in demos:
	# Read joint data
	for i in range(n_joints):
		joints[i].append_data(get_joint_data(filename,i+1)[0])
		
	# Read cartesian data
	cartesian_data.append(get_cartesian_data(filename))
	
### Movement primitives

rbf_width = 1e-4
n_rbfs_per_second = 50 / 0.7

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


