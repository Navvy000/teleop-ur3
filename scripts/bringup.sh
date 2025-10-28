#!/usr/bin/env bash
set -e

# Allow users to export ROS_WS before running to customize workspace; default is ~/ros2_ws_1
ROS_WS="${ROS_WS:-$HOME/ros2_ws_1}"

# 1) Python venv
source "$HOME/ros2venv/bin/activate"

# 2) ROS 2
source /opt/ros/jazzy/setup.bash

# 3) Merge Python paths of venv and ROS 2
export PYTHONPATH="$VIRTUAL_ENV/lib/python3.12/site-packages:/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"

# 4) Workspace environment (if already built)
if [ -f "$ROS_WS/install/setup.bash" ]; then
  source "$ROS_WS/install/setup.bash"
fi

echo "[OK] Env loaded. ROS_WS=$ROS_WS"
