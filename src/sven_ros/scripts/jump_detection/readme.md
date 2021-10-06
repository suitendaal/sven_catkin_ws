# Jump detection

Jump detection is the offline algorithm that detects the time instances of the impacts and uses them to create Reference Spreading trajectories based on demonstrations.

## Installation
Install [ROS](http://wiki.ros.org/melodic/Installation).
Then source ROS

```bash
source /path/to/ros/version/setup.bash
```

or

```bash
source devel/setup.bash
```

## Usage

### Analyze bound and window length

The script `analyze_bound.py` is used to analyze the bound and the window length of the Jump Aware filter.
Place the demonstration files in the folder `data` and edit the configuration file `config/config_jump_detection.py` such that the names of the demonstration files are read.
The script analyzes for different window lengths for the Jump Aware filter what the ideal bound is and then calculates a score for each of the window lengths.
The window length with the lowest score is considered the optimal window length.

### Identify outlier demonstration files

The script `identify_outlier_demos.py` analyzes each of the scripts and determines whether the data can be used in the detection of jumps and the trajectory creation or not.
It also analyzes the ideal bound using only the valid demonstration files.
Before starting this script, set the desired window length in the file `config/config_jump_detection.py`.

### Detect jumps

The script `detect_jumps.py` detects the impact and jump times of the demonstration files.
The jumps and impacts are detected based on the external force data, by making predictions of the value of the datapoints and comparing these predictions against the actual values of the datapoints.
A jump is considered an impact if the value of the datapoint is larger than the predicted value.
Before starting this script, fill in the desired parameters in `config/config_jump_detection.py`.

### Impact detection delay

The script `impact_detection_delay.py` analyzes the demonstration files around the time of impact to determine the delay of the causal jump detector.
This is done by looking at the difference between the external force data and the predictions made by the jump detector.
From the point where the impact is detected, the script looks backwards into the data to search for the first local minimum.
The difference between the timestamp of the detected impact and the timestamp of one datapoint after the local minimum is considered as the impact detection delay for that impact.
This procedure is done for both the first impact as the last impact of the practical-simultaneous impact interval.
Before running this script, fill in the desired parameters in `config/config_impact_detection_delay.py`

