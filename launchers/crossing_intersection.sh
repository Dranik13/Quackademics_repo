#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun followlane crossing_intersection_node.py

dt-launchfile-join