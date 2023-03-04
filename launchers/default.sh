#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------
#dt-exec roslaunch augmented_reality_basics augmented_reality_basics.launch map_file:="intersection_4way.yaml" veh:=csc22945
#dt-exec roslaunch ex3_project ex3_project.launch veh:=csc22945
dt-exec roslaunch led_emitter led_emitter_node.launch veh:=csc22945
dt-exec roslaunch duckietown_demos deadreckoning.launch
dt-exec roslaunch ex3_project ex3_project.launch veh:=csc22945


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# launching app

#dt-exec static_transform_publisher 0.5 0.5 0.5 0 0 0 base_footprint random1

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
