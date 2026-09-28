#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-26}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
mkdir -p "$ROOT/Webots/logs"
case "${1:-help}" in
  motion)
    export BEL_WEBOTS_AUTOTEST=1
    exec /usr/local/bin/webots --batch --minimize --no-rendering --mode=fast --stdout --stderr --port=1235 "$ROOT/Webots/worlds/bel_warehouse.wbt"
    ;;
  sim)
    source /opt/ros/jazzy/setup.bash
    exec /usr/local/bin/webots --batch --mode=realtime --stdout --stderr --port=1235 "$ROOT/Webots/worlds/bel_warehouse_ros.wbt"
    ;;
  fleet)
    source /opt/ros/jazzy/setup.bash
    source "$ROOT/ROS 2/install/setup.bash"
    exec ros2 launch bel_amr_ros webots_fleet.launch.py
    ;;
  test-interface|test-safety|test-navigation)
    source /opt/ros/jazzy/setup.bash
    case "$1" in
      test-interface) testfile=verify_ros_interface.py ;;
      test-safety) testfile=verify_lidar_stop.py ;;
      test-navigation) testfile=verify_navigation.py ;;
    esac
    exec python3 "$ROOT/Webots/tests/$testfile"
    ;;
  build)
    source /opt/ros/jazzy/setup.bash
    cd "$ROOT/ROS 2"
    exec colcon build --symlink-install --packages-select bel_amr_ros
    ;;
  *) echo 'Usage: bash tools/webots.sh {motion|sim|build|fleet|test-interface|test-safety|test-navigation}' ;;
esac
