#!/usr/bin/env bash
set -e

cd ~/robot2d_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting world_node..."
ros2 run world_sim world_node
