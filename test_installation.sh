#!/bin/bash
# Test the installation for the SO-101 Forward Kinematics & MoveIt Workspace
# Usage: ./test_installation.sh

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source "$SCRIPT_DIR/activate.sh"

echo ""
echo "========================================"
echo "Testing SO-101 Forward Kinematics & MoveIt Workspace Installation"
echo "========================================"
echo ""

# Test Python packages
echo "1. Testing Python packages..."
python3 -c "
import sys
print(f'   Python: {sys.version}')

import numpy as np
print(f'   NumPy: {np.__version__}')

import scipy
print(f'   SciPy: {scipy.__version__}')

try:
    import matplotlib
    print(f'   Matplotlib: {matplotlib.__version__}')
except ImportError:
    print('   Matplotlib: NOT INSTALLED')
"

# Test ROS2
echo ""
echo "2. Testing ROS2 & Workspace..."
if [ -n "$ROS_DISTRO" ]; then
    echo "   ROS2 Distro: $ROS_DISTRO"
    
    if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
        echo "   ROS2 Workspace: BUILT & Sourced"
    else
        echo "   ROS2 Workspace: NOT BUILT (run colcon build in ros2_ws)"
    fi
else
    echo "   ROS2: NOT INSTALLED OR NOT SOURCED"
fi

# Verify Lab 1.1 files
echo ""
echo "3. Verifying Lab 1.1 files..."
if [ -f "$SCRIPT_DIR/labs/lab1_1_visualize_urdf.py" ]; then
    echo "   ✓ lab1_1_visualize_urdf.py is present"
else
    echo "   ✗ lab1_1_visualize_urdf.py is MISSING!"
fi

if [ -f "$SCRIPT_DIR/ros2_ws/src/lerobot_description/urdf/so101.urdf" ]; then
    echo "   ✓ SO-101 URDF file is present"
else
    echo "   ✗ SO-101 URDF file is MISSING!"
fi

echo ""
echo "========================================"
echo "Installation test complete!"
echo "========================================"
