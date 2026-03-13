#!/usr/bin/env bash
set -e

cd ~/robot2d_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting goal_nav..."
ros2 run world_sim goal_nav
