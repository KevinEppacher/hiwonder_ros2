#!/usr/bin/env bash
cd /app
source /opt/ros/jazzy/setup.bash

echo "Building the workspace inside the container..."
colcon build --symlink-install
