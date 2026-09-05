#!/bin/bash
#===============================================================================
# SO-101 Forward Kinematics & MoveIt Workspace Setup Script
# ====================================
# 
# This script sets up the development environment for the SO-101 robotics labs.
#
# Target System:
#   - Ubuntu 22.04 LTS (with ROS2 Humble) or Ubuntu 24.04 LTS (with ROS2 Jazzy)
#   - Python 3.10+
#
# What this script does:
#   1. Checks system requirements (ROS2 Humble or Jazzy)
#   2. Installs system dependencies
#   3. Creates Python virtual environment
#   4. Installs Python packages (numpy, scipy, etc.)
#   5. Sells/Sourced pre-configured ROS2 workspace
#   6. Builds ROS2 packages in the local workspace using colcon
#   7. Creates convenience activation script
#
# Usage:
#   chmod +x setup.sh
#   ./setup.sh
#
#===============================================================================

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_DIR="$SCRIPT_DIR"

echo -e "${BLUE}"
echo "============================================================"
echo "  SO-101 Forward Kinematics & MoveIt Workspace Setup"
echo "============================================================"
echo -e "${NC}"

#===============================================================================
# Helper Functions
#===============================================================================

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_command() {
    if command -v "$1" &> /dev/null; then
        return 0
    else
        return 1
    fi
}

#===============================================================================
# Step 1: Check System Requirements
#===============================================================================

echo -e "\n${BLUE}[Step 1/7] Checking system requirements...${NC}\n"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$ID" == "ubuntu" ]]; then
        log_info "Detected: $PRETTY_NAME"
    else
        log_warn "This script is designed for Ubuntu. You have $ID."
    fi
else
    log_warn "Could not detect OS version."
fi

# Check Python
if check_command python3; then
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    log_info "Python version: $PYTHON_VERSION"
else
    log_error "Python3 not found. Please install Python 3.10+."
    exit 1
fi

# Check ROS2
ROS2_SETUP=""
if [ -f /opt/ros/jazzy/setup.bash ]; then
    log_info "ROS2 Jazzy found at /opt/ros/jazzy"
    ROS2_AVAILABLE=true
    ROS2_SETUP="/opt/ros/jazzy/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
    log_info "ROS2 Humble found at /opt/ros/humble"
    ROS2_AVAILABLE=true
    ROS2_SETUP="/opt/ros/humble/setup.bash"
else
    log_warn "ROS2 (Humble or Jazzy) setup.bash not found at /opt/ros/humble or /opt/ros/jazzy."
    log_warn "ROS2-based visualization will not work. Please install ROS2."
    ROS2_AVAILABLE=false
fi

# Check git
if check_command git; then
    log_info "Git is installed"
else
    log_error "Git not found. Installing..."
    sudo apt-get update && sudo apt-get install -y git
fi

#===============================================================================
# Step 2: Install System Dependencies
#===============================================================================

echo -e "\n${BLUE}[Step 2/7] Installing system dependencies...${NC}\n"

log_info "Updating package lists..."
sudo apt-get update

log_info "Installing required system packages..."
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-dev \
    build-essential \
    cmake \
    libglfw3 \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    libosmesa6-dev \
    patchelf \
    ffmpeg \
    libsm6 \
    libxext6 \
    libxrender-dev

log_info "System dependencies installed."

#===============================================================================
# Step 3: Create Python Virtual Environment
#===============================================================================

echo -e "\n${BLUE}[Step 3/7] Setting up Python virtual environment...${NC}\n"

VENV_DIR="$PROJECT_DIR/venv"

if [ -d "$VENV_DIR" ]; then
    log_warn "Virtual environment already exists at $VENV_DIR"
    read -p "Do you want to recreate it? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        log_info "Removing existing virtual environment..."
        rm -rf "$VENV_DIR"
        python3 -m venv "$VENV_DIR"
        log_info "Virtual environment recreated."
    fi
else
    log_info "Creating virtual environment at $VENV_DIR..."
    python3 -m venv "$VENV_DIR"
    log_info "Virtual environment created."
fi

# Activate virtual environment
log_info "Activating virtual environment..."
source "$VENV_DIR/bin/activate"

# Upgrade pip
log_info "Upgrading pip..."
pip install --upgrade pip

#===============================================================================
# Step 4: Install Python Dependencies
#===============================================================================

echo -e "\n${BLUE}[Step 4/7] Installing Python dependencies...${NC}\n"

log_info "Installing requirements..."
if [ -f "$PROJECT_DIR/requirements.txt" ]; then
    pip install -r "$PROJECT_DIR/requirements.txt"
else
    pip install numpy scipy matplotlib pyyaml
fi

# Install the project in editable mode
log_info "Installing so101_robotics_course package..."
pip install -e "$PROJECT_DIR"

log_info "Python dependencies installed."

#===============================================================================
# Step 5: Setup ROS2 Workspace
#===============================================================================

echo -e "\n${BLUE}[Step 5/7] Setting up ROS2 workspace...${NC}\n"

ROS2_WS="$PROJECT_DIR/ros2_ws"

if [ "$ROS2_AVAILABLE" = true ]; then
    # Source ROS2
    source "$ROS2_SETUP"
    
    log_info "Using pre-configured ROS2 workspace in: $ROS2_WS"
    log_info "ROS2 workspace setup complete."
else
    log_warn "Skipping ROS2 workspace setup (ROS2 not available)."
fi

#===============================================================================
# Step 6: Build ROS2 Packages
#===============================================================================

echo -e "\n${BLUE}[Step 6/7] Building ROS2 packages...${NC}\n"

if [ "$ROS2_AVAILABLE" = true ] && [ -d "$ROS2_WS/src" ]; then
    cd "$ROS2_WS"
    
    # Source ROS2 setup
    source "$ROS2_SETUP"
    
    log_info "Installing ROS2 dependencies with rosdep..."
    
    # Initialize rosdep if needed
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init || log_warn "rosdep already initialized"
    fi
    rosdep update || log_warn "rosdep update had warnings"
    
    # Install dependencies
    rosdep install --from-paths src --ignore-src -r -y || \
        log_warn "Some rosdep dependencies could not be installed"
    
    log_info "Building ROS2 workspace..."
    colcon build --symlink-install || {
        log_warn "colcon build failed. Some ROS2 features may not work."
        log_warn "Check the error messages above and try building manually."
    }
    
    log_info "ROS2 packages built."
    cd "$PROJECT_DIR"
else
    log_warn "Skipping ROS2 build (ROS2 not available or workspace not setup)."
fi

#===============================================================================
# Step 7: Create Convenience Scripts
#===============================================================================

echo -e "\n${BLUE}[Step 7/7] Creating convenience scripts...${NC}\n"

# Create activation script
ACTIVATE_SCRIPT="$PROJECT_DIR/activate.sh"
cat > "$ACTIVATE_SCRIPT" << EOF
#!/bin/bash
# Source this script to activate the SO-101 development environment
# Usage: source activate.sh

SCRIPT_DIR="\$( cd "\$( dirname "\${BASH_SOURCE[0]}" )" && pwd )"

# Activate Python virtual environment
if [ -f "\$SCRIPT_DIR/venv/bin/activate" ]; then
    source "\$SCRIPT_DIR/venv/bin/activate"
    echo "✓ Python virtual environment activated"
else
    echo "✗ Virtual environment not found. Run setup.sh first."
fi

# Source ROS2 setup
if [ -n "$ROS2_SETUP" ] && [ -f "$ROS2_SETUP" ]; then
    source "$ROS2_SETUP"
    echo "✓ ROS2 sourced from $ROS2_SETUP"
fi

# Source ROS2 workspace
if [ -f "\$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "\$SCRIPT_DIR/ros2_ws/install/setup.bash"
    echo "✓ ROS2 workspace sourced"
fi

# Add project to PYTHONPATH
export PYTHONPATH="\$SCRIPT_DIR/src:\$PYTHONPATH"

echo ""
echo "SO-101 Robotics Course environment activated!"
echo "Project directory: \$SCRIPT_DIR"
echo ""
EOF
chmod +x "$ACTIVATE_SCRIPT"
log_info "Created: activate.sh"

#===============================================================================
# Final Summary
#===============================================================================

echo -e "\n${BLUE}"
echo "============================================================"
echo "  Setup Complete!"
echo "============================================================"
echo -e "${NC}"

echo -e "${GREEN}What was installed:${NC}"
echo "  ✓ Python virtual environment (./venv)"
echo "  ✓ Python packages (numpy, scipy, matplotlib, pyyaml)"
echo "  ✓ Pre-configured ROS2 workspace (./ros2_ws)"

echo ""
echo -e "${GREEN}Quick Start:${NC}"
echo ""
echo "  1. Activate the environment:"
echo "     ${YELLOW}source activate.sh${NC}"
echo ""
echo "  2. Test the installation:"
echo "     ${YELLOW}./test_installation.sh${NC}"
echo ""
echo "  3. Run ROS2 URDF visualization:"
echo "     ${YELLOW}ros2 launch lerobot_description so101_display.launch.py${NC}"
echo ""
echo -e "${YELLOW}Note: Always run 'source activate.sh' before working on the labs!${NC}"
echo ""
