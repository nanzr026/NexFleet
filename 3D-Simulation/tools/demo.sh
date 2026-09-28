#!/usr/bin/env bash
set -e
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
# Share a lock across GUI/headless server modes and refuse existing publishers.
component="${1:-help}"
case "$component" in
  gazebo|headless) component=gazebo; pattern='^gz sim ' ;;
  bridge) pattern='/ros_gz_bridge/parameter_bridge ' ;;
  fleet) pattern='/bel_amr_ros/robot_controller ' ;;
  *) pattern='' ;;
esac
if [[ -n "$pattern" ]]; then
  exec 9>"/tmp/bel-amr-${UID}-${component}.lock"
  flock -n 9 || { echo "$component already running" >&2; exit 1; }
  if pgrep -f "$pattern" >/dev/null; then
    echo "Existing $component process detected; stop it before starting another." >&2
    exit 1
  fi
fi
case "${1:-help}" in
  build)
    cd "$ROOT/ROS 2"
    colcon build --symlink-install --packages-select bel_amr_ros
    ;;
  gazebo)
    export GZ_SIM_RESOURCE_PATH="$ROOT/Gazebo Harmonic/models:${GZ_SIM_RESOURCE_PATH:-}"
    exec gz sim -r "$ROOT/Gazebo Harmonic/worlds/openrobotics_warehouse.sdf"
    ;;
  headless)
    export GZ_SIM_RESOURCE_PATH="$ROOT/Gazebo Harmonic/models:${GZ_SIM_RESOURCE_PATH:-}"
    exec gz sim -s -r --headless-rendering "$ROOT/Gazebo Harmonic/worlds/openrobotics_warehouse.sdf"
    ;;
  bridge)
    source "$ROOT/ROS 2/install/setup.bash"
    exec ros2 launch bel_amr_ros three_amrs.launch.py start_bridge:=true start_controllers:=false
    ;;
  fleet)
    source "$ROOT/ROS 2/install/setup.bash"
    exec ros2 launch bel_amr_ros three_amrs.launch.py
    ;;
  federated)
    source "$ROOT/ROS 2/install/setup.bash"
    exec ros2 launch bel_amr_ros federated_demo.launch.py
    ;;
  backend)
    cd "$ROOT/Dashboard/backend"
    exec .venv/bin/python main.py
    ;;
  frontend)
    cd "$ROOT/Dashboard"
    exec npm run dev -- --host 127.0.0.1
    ;;
  *)
    printf '%s\n' 'Usage: bash tools/demo.sh {build|gazebo|headless|bridge|fleet|backend|frontend}'
    ;;
esac
