#!/bin/bash
source ${CATKIN_WS_DIR}/devel/setup.bash
set -exu pipefail

# export ROS_MASTER_URI=http://`hostname`:11311
# roslaunch template.launch &

roslaunch duckietown_demos lane_following.launch &
aido-ros-bridge-launch # --launch template.launch
