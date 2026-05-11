# LF ROS Template

This repository is the ente ROS submission template for `aido-LF-sim-validation`.
It bridges Duckiematrix world I/O into a minimal ROS graph and launches a placeholder random-action controller by default.

The Docker image installs `git`, `dtps-http`, and runtime calibration defaults during the image build.
When the evaluator enables SHM mode via `DTSHELL_SHM_PATH`, the bridge automatically switches to SHM world I/O. Otherwise it uses DTPS world I/O against the live Duckiematrix session.

The solution tree now contains a single ROS package rooted at `solution/`, with the Duckiematrix bridge at `solution/src/main.py` and the placeholder controller at `solution/src/random_action_node.py`.

To keep the repository layout aligned with `challenge-aido_LF-baseline-duckietown`, the template also carries a `requirements.txt` placeholder, a `solution/README.md`, and launcher path names for the baseline-only lane-following flow. Those extra paths are structural scaffolding only; the template still ships the placeholder random-action behavior by default.

The template still includes this minimal runnable `solution/` because a submission template needs to build and run end-to-end against the evaluator while leaving the actual controller behavior to users and baselines.

The Dockerfile expects local BuildKit contexts for `duckietown-messages`, `duckietown-sdk`, and `dt-duckiematrix`. Use the baseline repo when you want the stock ROS lane-following stack instead of the placeholder template behavior.
