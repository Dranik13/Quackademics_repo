#!/bin/bash

source /environment.sh

dt-launchfile-init

rosrun followlane WhiteMaskDebugNode.py

dt-launchfile-join