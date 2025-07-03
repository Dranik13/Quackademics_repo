#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun followlane control_traffic_intersection_node.py

dt-launchfile-join