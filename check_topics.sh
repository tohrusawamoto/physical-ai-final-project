#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source "$HOME/airobot_ws/install/setup.bash"
exec ros2 topic info /cmd_vel
