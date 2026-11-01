#!/usr/bin/env bash
# Run inside the Ubuntu desktop provided by the course Docker container.
set -e
if [ ! -r /opt/ros/humble/setup.bash ] || [ ! -r "$HOME/airobot_ws/install/setup.bash" ]; then
  echo 'ROS 2 Humble or the course workspace was not found. Run this inside the Ubuntu container.' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
source "$HOME/airobot_ws/install/setup.bash"
export TURTLEBOT3_MODEL=happy_mini
exec ros2 launch turtlebot3_gazebo empty_world.launch.py
