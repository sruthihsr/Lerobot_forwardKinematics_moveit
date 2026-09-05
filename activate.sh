#!/bin/bash
# Source this script to activate the SO-101 development environment
# Usage: source activate.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Activate Python virtual environment
if [ -f "$SCRIPT_DIR/venv/bin/activate" ]; then
    source "$SCRIPT_DIR/venv/bin/activate"
    echo "✓ Python virtual environment activated"
else
    echo "✗ Virtual environment not found. Run ./setup.sh first."
fi

# Auto-detect and source ROS2 (Jazzy or Humble)
if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
    echo "✓ ROS2 Jazzy sourced"
elif [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
else
    echo "⚠ ROS2 setup.bash not found in /opt/ros/humble or /opt/ros/jazzy"
fi

# Source ROS2 workspace if built
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
    echo "✓ ROS2 workspace sourced"
else
    echo "⚠ ROS2 workspace not built yet. Run ./setup.sh or run 'colcon build' in 'ros2_ws'"
fi

# Add project to PYTHONPATH
export PYTHONPATH="$SCRIPT_DIR/src:$PYTHONPATH"

echo ""
echo "SO-101 Robotics Course environment activated!"
echo "Project directory: $SCRIPT_DIR"
echo ""
