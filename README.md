# LF ROS Template

This repository is the ente ROS submission template for `aido-LF-sim-validation`.
It bridges Duckiematrix world I/O into a minimal ROS graph and launches a placeholder random-action controller by default.

The Docker image installs its Duckietown Python runtime dependencies from `dependencies.txt`, pulling the maintained repositories from GitHub instead of copying from local source trees, and then applies the runtime calibration defaults during the image build.
When the evaluator enables SHM mode via `DTSHELL_SHM_PATH`, the bridge automatically switches to SHM world I/O. Otherwise it uses DTPS world I/O against the live Duckiematrix session.

The solution tree now contains a single ROS package rooted at `solution/`, with the Duckiematrix bridge at `solution/src/main.py` and the placeholder controller at `solution/src/random_action_node.py`.

To keep the repository layout aligned with `challenge-aido_LF-baseline-duckietown`, the template carries the same launcher and calibration paths while still shipping the placeholder random-action behavior by default.

The template still includes this minimal runnable `solution/` because a submission template needs to build and run end-to-end against the evaluator while leaving the actual controller behavior to users and baselines.

The Dockerfile installs the maintained Duckietown Python repositories from `dependencies.txt` instead of local BuildKit contexts. Use the baseline repo when you want the stock ROS lane-following stack instead of the placeholder template behavior.
