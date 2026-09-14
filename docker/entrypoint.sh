#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /opt/tare_planner/devel/setup.bash

# TARE links against the official OR-Tools release shipped with its source.
# Keep this path ahead of system libraries for both amd64 and arm64 images.
export LD_LIBRARY_PATH="/opt/tare_planner/src/tare_planner/src/tare_planner/or-tools/lib:/opt/tare_planner/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/tare_planner/devel/lib:/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_PACKAGE_PATH="/opt/tare_planner/src:/opt/ros/noetic/share:${ROS_PACKAGE_PATH:-}"

cd /opt/tare_planner
exec "$@"
