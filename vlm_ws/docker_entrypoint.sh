#!/bin/bash
set -e

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

# Export TurtleBot3 model
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}

# If a command was provided, run it (useful for `docker run <image> <command>`)
if [ "$#" -gt 0 ]; then
  exec "$@"
else
  # otherwise, give an interactive shell
  exec bash
fi
