#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun followlane detect_traffic_intersection_node.py

dt-launchfile-join