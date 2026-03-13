#!/usr/bin/env bash
set -e

cd ~/robot2d_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Fix for blank matplotlib window on some Ubuntu/Wayland setups:
export QT_QPA_PLATFORM=xcb

echo "Starting viewer..."
python3 -m world_sim.viewer
