#!/usr/bin/env bash
# Usage: bash run_demo.sh commands.json [--validate-only]
set -e
if [ ! -r /opt/ros/humble/setup.bash ] || [ ! -r "$HOME/airobot_ws/install/setup.bash" ]; then
  echo 'ROS 2 Humble or the course workspace was not found. Run this inside the Ubuntu container.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$HOME/airobot_ws/install/setup.bash"
exec python3 run_robot.py "$@"
