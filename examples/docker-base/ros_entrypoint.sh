#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Copyright (C) 2025 Physical AI & Humanoid Robotics Book Contributors

set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute command
exec "$@"
