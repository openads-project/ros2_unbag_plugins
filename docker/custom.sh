#!/usr/bin/env bash
set -e

# Source the ROS 2 workspace
source /opt/ros/${ROS_DISTRO}/setup.bash
[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash

# Install OpenADS routines and processors if provided
shopt -s nullglob
for routine in /docker-ros/additional-files/routines/*.py; do
  ros2 unbag --install-routine "$routine"
done
for processor in /docker-ros/additional-files/processors/*.py; do
  ros2 unbag --install-processor "$processor"
done
