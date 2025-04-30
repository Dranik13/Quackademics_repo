#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#dt-exec python3 -m "my_package.my_script"
#echo "Inhalt dieses Verzeichnisses:"
#ls -l "$(dirname "$0")/launch/"

#ls -l /launch/Quackademics_repo/control-lane.sh
#echo "Inhalt von $DT_REPO_PATH/launchers:"
#ls -l "$(dirname "$0")/launchers"
dt-exec bash /launch/Quackademics_repo/camera-reader.sh &
dt-exec bash /launch/Quackademics_repo/control-lane.sh &
dt-exec bash /launch/Quackademics_repo/control-obstacle.sh &
dt-exec bash /launch/Quackademics_repo/detect-duckie.sh &
dt-exec bash /launch/Quackademics_repo/detect-lane.sh &
dt-exec bash /launch/Quackademics_repo/switch-control.sh

#dt-exec bash "./dt-launcher-control-lane"

#dt-launcher-control-lane
#dt-exec bash "$DT_REPO_PATH/launchers/detect-lane.sh"
#dt-exec rosrun followlane camera_reader_node.py
#dt-exec rosrun followlane control_lane_node.py
#dt-exec rosrun followlane control_obstacle_node.py
#dt-exec rosrun followlane detect_duckie_node.py
#dt-execrosrun followlane detect_lane_node.py
#dt-exec rosrun followlane switch_control_node.py
# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
