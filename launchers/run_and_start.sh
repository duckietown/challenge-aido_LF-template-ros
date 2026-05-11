#!/bin/bash

source /environment.sh
source /opt/ros/noetic/setup.bash

set -euxo pipefail

source_if_present() {
    local setup_script="$1"

    if [[ -n "${setup_script}" && -f "${setup_script}" ]]; then
        source "${setup_script}" --extend
    fi
}

main() {
    local status=0

    source_if_present "${CATKIN_WS_DIR:-}/devel/setup.bash"
    source_if_present "/code/solution/devel/setup.bash"

    dt-exec-BG roscore
    dt-exec-BG roslaunch --wait agent random_action_node.launch

    set +e
    dt-exec-FG roslaunch --wait agent agent_node.launch
    status=$?
    set -e

    copy-ros-logs
    return "${status}"
}

main "$@"
