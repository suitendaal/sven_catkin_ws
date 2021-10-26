#!/usr/bin/python3

from trajectory_designer import *
from datalib import *
from .config_plot_figures import *
import math

#### Input settings

frequency = 100
designer = TrajectoryDesigner(frequency)

via_points = PoseDataSet([
	PoseDataPoint(0, [0.30, 0.54, 0.2, math.pi, 0, math.pi/2]),
	PoseDataPoint(3, [0.30, 0.54, 0.2, math.pi, 0, math.pi/2]),
	PoseDataPoint(6, [0.30, 0.54, 0, math.pi, 0, math.pi/2]),
	PoseDataPoint(9, [0.30, 0.54, 0, math.pi, 0, math.pi/2])
])

#### Output settings

## Output filename
designed_trajectory_output_file = 'output/designed_trajectory.json'
variable_labels = ['X','Y','Z',r'$\phi$',r'$\theta$',r'$\psi$']


