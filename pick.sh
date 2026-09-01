#!/bin/bash
# Usage: ./pick.sh <red|green|blue>
set -e

COLOR="${1:?Usage: ./pick.sh <red|green|blue>}"

echo "Stopping any existing pick_and_sort_node..."
pkill -9 -f "pick_and_sort_node" 2>/dev/null || true
sleep 1

echo "Starting pick_and_sort_node..."
ros2 run franka_color_sort pick_and_sort_node --ros-args -p use_sim_time:=true &
NODE_PID=$!

echo "Waiting for pick_and_sort_node service to become available..."
until ros2 service list 2>/dev/null | grep -q "/pick_and_sort_node/pick_color"; do
    sleep 0.5
done

echo "Calling pick_color for '$COLOR'..."
ros2 service call /pick_and_sort_node/pick_color \
    franka_color_sort_interfaces/srv/PickColor "{color: '$COLOR'}"

echo "Done. pick_and_sort_node (pid $NODE_PID) is still running in the background;"
echo "it will be killed automatically the next time you run this script."
