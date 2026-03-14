#!/usr/bin/env bash
#
# Push all visual_servoing files to the robot via scp.
#
# Usage:
#   ./scripts/deploy.sh           # defaults to car 102
#   ./scripts/deploy.sh 104       # specify car number
#
# After this finishes, SSH into the robot, connect to Docker, and run:
#   cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash

set -euo pipefail

CAR="${1:-102}"
ROBOT="racecar@192.168.1.${CAR}"
REMOTE_WS="~/racecar_ws/src"
LOCAL_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "==> Deploying to ${ROBOT}"
echo ""

# visual_servoing ROS2 package
echo "  visual_servoing/"
scp -r "${LOCAL_ROOT}/visual_servoing/" "${ROBOT}:${REMOTE_WS}/visual_servoing/"

# vs_msgs custom message definitions
echo "  vs_msgs/"
scp -r "${LOCAL_ROOT}/vs_msgs/" "${ROBOT}:${REMOTE_WS}/vs_msgs/"

echo ""
echo "==> Files deployed. Now on the robot (inside Docker):"
echo ""
echo "    cd ~/racecar_ws && colcon build --symlink-install && source install/setup.bash"
echo ""
echo "==> To run cone parking (with teleop already running in another terminal):"
echo ""
echo "    ros2 launch visual_servoing parking_deploy.launch.xml"
echo ""
echo "==> To run YOLO annotator (with ZED camera already running):"
echo ""
echo "    ros2 launch visual_servoing yolo_annotator.launch.xml"
echo ""
