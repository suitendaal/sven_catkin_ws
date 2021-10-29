#!/usr/bin/python3

from trajectory_designer import *
from .config_plot_figures import *

#### Input settings

frequency = 100
designer = TrajectoryDesigner(frequency)

from .trajectories.trajectory2 import via_points

#### Output settings

## Output filename
designed_trajectory_output_file = 'output/designed_trajectory.json'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']


