#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then source install/setup.bash; fi

usage() {
  cat <<'USAGE'
Usage: bash src/arm/scripts/arm.sh <mock|sim>
  mock  Start Aubo i5 with aubo_i5_mock.yaml and ROS-only interface.
  sim   Connect the arm service to Gazebo controllers (start Gazebo first).

Do not run a standalone arm with Core managing the same device ID.
USAGE
}

profile="${1:-help}"
if [[ $# -gt 1 ]]; then echo "This script does not accept extra launch arguments." >&2; exit 2; fi
case "$profile" in
  mock)
    exec ros2 launch embodied_nexus_arm arm.launch.py \
      config_file:=src/arm/embodied_nexus_arm/config/aubo_i5_mock.yaml interface_mode:=ros2 use_sim_time:=false
    ;;
  sim|simulation)
    exec ros2 launch embodied_nexus_arm arm.launch.py \
      config_file:=src/arm/embodied_nexus_arm/config/aubo_i5_sim.yaml interface_mode:=ros2 use_sim_time:=true
    ;;
  help|-h|--help) usage ;;
  *) echo "Profile must be mock or sim." >&2; usage >&2; exit 2 ;;
esac
