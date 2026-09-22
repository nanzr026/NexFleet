#!/bin/bash
echo "Building ROS 2 Workspace for NEXFLEET..."
cd ros2_ws || exit
colcon build --symlink-install
echo "Build complete. Run 'source install/setup.bash' to use."
