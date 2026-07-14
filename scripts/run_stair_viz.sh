#!/usr/bin/env bash
# Visualize stair/transition detection from a rosbag in RViz2.
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BAG="${1:-$ROOT/rosbags/stairs_case_1}"
RATE="${RATE:-2.0}"

# ROS setup.bash references optional unset vars; keep nounset off while sourcing.
set +u
source ~/unitree_ros2/setup.sh
source "$ROOT/install/setup.bash"
set -u

echo "Bag: $BAG"
echo "Orange = stair/transition, Cyan = support surface, Intensity = obstacles"
echo "Ctrl+C to stop."

rviz2 -d "$ROOT/rviz/stair_transition.rviz" >/tmp/stair_rviz.log 2>&1 &
RVIZ_PID=$!
trap 'kill $RVIZ_PID 2>/dev/null || true' EXIT

sleep 1
python3 "$ROOT/scripts/viz_stair_transition.py" --bag "$BAG" --rate "$RATE" --loop
