#!/bin/bash

source /environment.sh

set -euo pipefail

main() {
    printf '%s\n' \
        "Lane-following launchers are placeholders in challenge-aido_LF-template-ros." \
        "Use challenge-aido_LF-baseline-duckietown for the stock lane-following stack."
}

main "$@"
