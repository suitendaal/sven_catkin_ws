This ROS workspace is used for my [thesis](https://repository.tudelft.nl/record/uuid:0a2f6f1e-3a07-41ec-a2b2-64558685b530) called _Teaching Robots Impact Tasks by Performing Demonstrations_.
The code was developed as part of an experimental research setup and is provided for reference and documentation purposes only.

## Project overview
The goal of this work was to investigate how impact-based manipulation tasks can be taught to a robot through human demonstrations.
A Franka Emika Panda robot is teleoperated using an HTC Vive controller, allowing a human demonstrator to perform task executions involving impact.
These demonstrations are then used to transfer the skill to the robot manipulator.

## My contribution
* Designed and implemented 4 custom ROS controllers described in the thesis.
* Developed a teleoperation pipeline using HTC Vive input.
* Implemented a detector for impacts.
The core implementation can be found in the package `sven_ros`.

## Technical stack
* [ROS Melodic](https://wiki.ros.org/melodic)
* [franka_ros](https://github.com/frankarobotics/franka_ros/tree/melodic-devel) and libfranka
* [vive_ros](https://github.com/robosavvy/vive_ros)
* Steam VR
* HTC Vive controller (2018 version)

## Repository structure
`sven_ros/`: ROS package containing the controllers, pipeline and impact detector.
`scripts/`: Helper scripts for connecting to the robot and launching ROS nodes.

## Important note on reproducibility
This workspace was developed for a specific hardware setup and ROS installation.
Scripts and configurations contain system-specific assumptions (e.g. network setup and paths).

In particular:
* This repository is not intended as a generic ROS package.
* `scripts/connect_to_panda.sh` must be adapted for other systems.

The code is shared to support the thesis and demonstrate system design and implementation choices.

## Reference
If you use this work, please cite the thesis linked above.
