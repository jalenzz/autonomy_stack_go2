#!/bin/bash

set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

cd "$SCRIPT_DIR"
source "$HOME/unitree_ros2/setup.sh"
source "$HOME/ws_livox/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"
exec ros2 launch vehicle_simulator system_real_robot_with_route_planner.launch
