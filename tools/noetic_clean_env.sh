#!/usr/bin/env bash
set -euo pipefail

# Run ROS Noetic commands with a sanitized runtime environment so they don't
# accidentally pick up conda/ROS 2 shared libraries from the container shell.

if [ "$#" -eq 0 ]; then
  echo "usage: $0 <command> [args...]" >&2
  exit 2
fi

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV
unset CONDA_EXE
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset PYTHONHOME

export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Force a known-good runtime library search order for system Python, ROS, BLAS
# and catkin outputs, while excluding conda and ROS 2 overlays.
export LD_LIBRARY_PATH="/catkin_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/opt/ros/noetic/share/euslisp/jskeus/eus//Linux64/lib:/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"
export PYTHONPATH="/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages"

exec "$@"
