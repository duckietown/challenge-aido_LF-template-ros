#!/bin/bash

roscore &
source /environment.sh
source /opt/ros/noetic/setup.bash
source /code/catkin_ws/devel/setup.bash --extend
source /code/submission_ws/devel/setup.bash --extend

roslaunch --wait agent agent_node.launch &
roslaunch --wait random_action random_action_node.launch
