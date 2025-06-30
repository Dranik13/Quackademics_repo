#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# Kamera-Node mit Save-Trigger starten
rosrun followlane camera_reader_node_save_trigger.py

# warte, bis Node beendet wird
dt-launchfile-join
