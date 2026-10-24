#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi

usage() {
  cat <<'USAGE'
Usage: bash src/base/scripts/base.sh <mock|sim>
  mock  Start the Scout base with scout_mock.yaml and ROS-only interface.
  sim   Connect the base service to Gazebo topics (start Gazebo first).

Do not run a standalone base with Core managing the same device ID.
USAGE
}

profile="${1:-help}"
if [[ $# -gt 1 ]]; then echo "This script does not accept extra launch arguments." >&2; exit 2; fi
case "$profile" in
  mock)
    exec ros2 launch embodied_nexus_base base.launch.py \
      config_file:=src/base/embodied_nexus_base/config/scout_mock.yaml interface_mode:=ros2 use_sim_time:=false
    ;;
  sim|simulation)
    exec ros2 launch embodied_nexus_base base.launch.py \
      config_file:=src/base/embodied_nexus_base/config/scout_sim.yaml interface_mode:=ros2 use_sim_time:=true
    ;;
  help|-h|--help) usage ;;
  *) echo "Profile must be mock or sim." >&2; usage >&2; exit 2 ;;
esac
