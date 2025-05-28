#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun followlane cmd_control.py

# wait for app to end
dt-launchfile-join