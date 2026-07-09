#!/bin/bash
# ============================================================
# Submarine Driver Framework - One-Click Demo Setup
# ============================================================
# Run this script on Ubuntu 22.04 with ROS2 Humble installed.
#
# What it does:
#   1. Creates a ROS2 workspace
#   2. Clones the repo (or uses existing)
#   3. Installs dependencies
#   4. Builds everything
#   5. Runs the demo (trajectory sim + state estimator + RViz)
#
# Usage:
#   chmod +x demo_setup.sh
#   ./demo_setup.sh
#
# Prerequisites:
#   - Ubuntu 22.04
#   - ROS2 Humble installed (see https://docs.ros.org/en/humble/Installation.html)
# ============================================================

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Submarine Driver Framework - Demo Setup         ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"

# Check ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Sourcing ROS2 Humble...${NC}"
    source /opt/ros/humble/setup.bash 2>/dev/null || {
        echo -e "${RED}ERROR: ROS2 Humble not found. Install it first:${NC}"
        echo "https://docs.ros.org/en/humble/Installation.html"
        exit 1
    }
fi
echo -e "${GREEN}✓ ROS2 $ROS_DISTRO detected${NC}"

# Create workspace
WS_DIR="$HOME/submarine_ws"
echo -e "\n${YELLOW}[1/5] Creating workspace at $WS_DIR${NC}"
mkdir -p "$WS_DIR/src"
cd "$WS_DIR/src"

# Clone or update repo
if [ -d "submarine-driver-framework" ]; then
    echo -e "${GREEN}✓ Repo already exists, pulling latest...${NC}"
    cd submarine-driver-framework && git pull && cd ..
else
    echo -e "${YELLOW}Cloning repo...${NC}"
    git clone https://github.com/OmTilwar/submarine-driver-framework.git
fi

# Install dependencies
echo -e "\n${YELLOW}[2/5] Installing dependencies...${NC}"
cd "$WS_DIR"
sudo apt-get update -qq
sudo apt-get install -y -qq ros-humble-robot-state-publisher ros-humble-rviz2 \
    ros-humble-tf2-ros python3-colcon-common-extensions 2>/dev/null
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true
echo -e "${GREEN}✓ Dependencies installed${NC}"

# Build
echo -e "\n${YELLOW}[3/5] Building with colcon...${NC}"
source /opt/ros/humble/setup.bash
colcon build --packages-select submarine_drivers \
    --cmake-args -DBUILD_ROS2_NODES=ON -DBUILD_TESTING=ON
echo -e "${GREEN}✓ Build complete${NC}"

# Run standalone tests
echo -e "\n${YELLOW}[4/5] Running unit tests...${NC}"
source install/setup.bash
colcon test --packages-select submarine_drivers
colcon test-result --verbose 2>/dev/null || true
echo -e "${GREEN}✓ Tests complete${NC}"

# Launch demo
echo -e "\n${YELLOW}[5/5] Launching demo...${NC}"
echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  Demo is starting! You should see:               ║${NC}"
echo -e "${GREEN}║    - RViz with submarine model                   ║${NC}"
echo -e "${GREEN}║    - Green arrow: estimated pose                 ║${NC}"
echo -e "${GREEN}║    - Yellow arrow: ground truth                  ║${NC}"
echo -e "${GREEN}║    - Submarine moving in a circle at 20m depth   ║${NC}"
echo -e "${GREEN}║                                                  ║${NC}"
echo -e "${GREEN}║  Press Ctrl+C to stop.                           ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}Tip: To record a demo video, use:${NC}"
echo "  sudo apt install kazam && kazam  # screen recorder"
echo ""

source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"
ros2 launch submarine_drivers demo.launch.py pattern:=circle
