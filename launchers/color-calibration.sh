#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# Starte white_yellow_calibration.py
dt-exec python3 "$DT_REPO_PATH/packages/followlane/src/white_yellow_calibration.py"

# warte, bis Node beendet wird
dt-launchfile-join