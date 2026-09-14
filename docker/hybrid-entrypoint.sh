#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /opt/tare_planner/devel/setup.bash
source /opt/gbplanner_ws/devel/setup.bash

export LD_LIBRARY_PATH="/opt/gbplanner_ws/devel/lib:/opt/tare_planner/src/tare_planner/src/tare_planner/or-tools/lib:/opt/tare_planner/devel/lib:/opt/ros/noetic/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="/opt/tare_planner/devel/lib:/opt/ros/noetic/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_PACKAGE_PATH="/opt/gbplanner_ws/src:/opt/tare_planner/src:/opt/ros/noetic/share:${ROS_PACKAGE_PATH:-}"

cd /opt/gbplanner_ws
exec "$@"
