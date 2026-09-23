#!/bin/bash
# Setup script for Ridgeback AMR Simulation

set -e

echo "🤖 Ridgeback AMR Simulation Setup"
echo "=================================="

# Check ROS2 installation
if [ ! -d "/opt/ros/humble" ]; then
    echo "❌ ROS2 Humble not found at /opt/ros/humble"
    echo "Please install ROS2 Humble first:"
    echo "  https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

echo "✅ ROS2 Humble detected"

# Source ROS2
source /opt/ros/humble/setup.bash

# Check dependencies
echo "📦 Checking dependencies..."

deps=(
    "ignition-fortress"
    "ros-humble-gazebo-ros-pkgs"
    "ros-humble-ros-gzbridge"
    "ros-humble-topic-tools"
)

missing_deps=()
for dep in "${deps[@]}"; do
    if ! dpkg -l | grep -q "$dep"; then
        missing_deps+=("$dep")
    fi
done

if [ ${#missing_deps[@]} -gt 0 ]; then
    echo "⚠️  Missing dependencies:"
    for dep in "${missing_deps[@]}"; do
        echo "    - $dep"
    done
    read -p "Install missing packages? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt-get update
        sudo apt-get install -y "${missing_deps[@]}"
    fi
fi

# Build the workspace
echo ""
echo "🔨 Building ROS2 packages..."
cd "$(dirname "$0")"

if command -v colcon &> /dev/null; then
    colcon build --symlink-install
else
    echo "❌ colcon not found. Install with:"
    echo "   sudo apt install python3-colcon-common-extensions"
    exit 1
fi

echo ""
echo "✅ Build complete!"
echo ""
echo "📝 Next steps:"
echo "  1. Source the workspace:"
echo "     source install/setup.bash"
echo ""
echo "  2. Launch the simulation:"
echo "     ros2 launch ridgeback_sim_launch ridgeback_sim.launch.py"
echo ""
echo "  3. In another terminal, run control nodes:"
echo "     ros2 run ridgeback_sim_control velocity_commander interactive"
echo ""
echo "For more details, see QUICKSTART.md"
