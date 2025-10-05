#!/bin/bash
# Start micro-ROS agent for Pico

# Source ROS 2
source /opt/ros/humble/setup.bash

# Start the agent with verbose output
exec sg dialout -c "micro-ros-agent serial --dev /dev/ttyACM0 -b 115200 -v6"
