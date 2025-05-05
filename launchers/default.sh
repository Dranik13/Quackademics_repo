#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------

# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app
#echo "Inhalt dieses Verzeichnisses:"
#ls -l "$(dirname "$0")/"

#dt-exec bash /launch/Quackademics_repo/camera-reader.sh 
dt-exec bash /launch/Quackademics_repo/control-lane.sh &
#dt-exec bash /launch/Quackademics_repo/control-obstacle.sh &
#dt-exec bash /launch/Quackademics_repo/detect-duckie.sh &
dt-exec bash /launch/Quackademics_repo/detect-lane.sh &
dt-exec bash /launch/Quackademics_repo/switch-control.sh
#dt-exec python3 -m "my_package.main"

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
