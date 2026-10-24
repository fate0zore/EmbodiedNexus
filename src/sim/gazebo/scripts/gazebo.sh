#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../../.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi

usage() {
  cat <<'USAGE'
Usage: bash src/sim/gazebo/scripts/gazebo.sh <build|start|headless>
  build     Build Gazebo integration and its dependencies.
  start     Start Fortress + Core with the WSLg GUI.
  headless  Start Fortress + Core without the GUI.

Set EMBODIED_NEXUS_CORE_TOKEN before start. IGN_IP defaults to the WSL eth0 IPv4
address; override it when Windows must reach a different WSL interface address.
Stop the foreground launch with Ctrl+C. Ctrl+Z only suspends it.
USAGE
}

action="${1:-help}"
if [[ $# -gt 1 ]]; then echo "This script does not accept extra launch arguments." >&2; exit 2; fi
case "$action" in
  build)
    colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_sim_gazebo
    if [[ -f install/setup.bash ]]; then source install/setup.bash; fi
    ;;
  start|headless)
    if [[ -z "${EMBODIED_NEXUS_CORE_TOKEN:-}" ]]; then
      echo "Set EMBODIED_NEXUS_CORE_TOKEN in this Bash environment before starting the simulation." >&2
      exit 2
    fi
    if [[ -z "${IGN_IP:-}" ]]; then
      IGN_IP="$(ip -4 -o addr show dev eth0 2>/dev/null | awk 'NR == 1 { split($4, a, "/"); print a[1] }')"
    fi
    if [[ -z "${IGN_IP:-}" ]]; then
      echo "Set IGN_IP to the WSL IPv4 address reachable from Windows." >&2
      exit 2
    fi
    export IGN_IP
    export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
    if [[ "$action" == start ]]; then
      export DISPLAY="${DISPLAY:-:0}"
      export XDG_RUNTIME_DIR=/tmp/embodied-nexus-gui-runtime
      mkdir -p "$XDG_RUNTIME_DIR"
      chmod 700 "$XDG_RUNTIME_DIR"
      export QT_QPA_PLATFORM=xcb
      unset WAYLAND_DISPLAY
      gui=true
    else
      gui=false
    fi
    exec ros2 launch embodied_nexus_sim_gazebo sim.launch.py \
      "gui:=$gui" "core_token:=$EMBODIED_NEXUS_CORE_TOKEN"
    ;;
  help|-h|--help) usage ;;
  *) echo "Action must be build, start or headless." >&2; usage >&2; exit 2 ;;
esac
