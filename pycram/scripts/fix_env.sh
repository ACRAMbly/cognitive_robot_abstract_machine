#!/bin/bash
set -e

echo "--------------------------------------------------------"
echo " Fixing Environment & Build Issues"
echo "--------------------------------------------------------"

# 1. Cleaning Stale .pth files (Fixes ImportErrors/Circular Imports)
#    These point to the Trash or old locations, confusing Python.
echo "[1/3] Removing stale .pth and .egg-link files..."
find ~/.virtualenvs/cram-env/lib/python*/site-packages/ -name "*.pth" -print0 | xargs -0 grep -l "Trash" | xargs -r rm -v
find ~/.virtualenvs/cram-env/lib/python*/site-packages/ -name "*.egg-link" -print0 | xargs -0 grep -l "Trash" | xargs -r rm -v
find ~/.local/lib/python*/site-packages/ -name "*.pth" -print0 | xargs -0 grep -l "Trash" | xargs -r rm -v
find ~/.local/lib/python*/site-packages/ -name "*.egg-link" -print0 | xargs -0 grep -l "Trash" | xargs -r rm -v

# 2. Clean build artifacts for problem packages
echo "[2/3] Cleaning build artifacts..."
cd ~/workspace/ros
rm -rf build/random_events install/random_events
rm -rf build/giskardpy_ros install/giskardpy_ros
rm -rf build/krrood install/krrood

# 3. Rebuild (Fixes absolute path error in random_events)
echo "[3/3] Rebuilding workspace (this may take a minute)..."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select random_events giskardpy_ros --event-handlers console_direct+

echo "--------------------------------------------------------"
echo " Done! Please verify by running:"
echo "   source ~/workspace/ros/install/setup.bash"
echo "   ros2 launch giskardpy_ros giskardpy_tracy_standalone.launch.py"
echo "--------------------------------------------------------"
