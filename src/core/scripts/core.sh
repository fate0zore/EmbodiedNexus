#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
if [[ -f install/setup.bash ]]; then
  source install/setup.bash
fi

usage() {
  cat <<'USAGE'
Usage:
  bash src/core/scripts/core.sh build [core|sim]
  bash src/core/scripts/core.sh start-mock
  bash src/core/scripts/core.sh status
  bash src/core/scripts/core.sh device <start|restart|shutdown> <device_id>
  bash src/core/scripts/core.sh batch <start_all|stop_all|shutdown_all|shutdown_core>

Core starts with devices stopped. start-mock requires EMBODIED_NEXUS_CORE_TOKEN.
Foreground Core exits with Ctrl+C; Ctrl+Z only suspends it.
USAGE
}

command_name="${1:-help}"
case "$command_name" in
  build)
    target="${2:-core}"
    case "$target" in
      core)
        rviz_dir="$REPO_ROOT/src/core/embodied_nexus_core/web_rviz"
        if [[ "${3:-}" == "--frontend-ready" ]]; then
          if [[ ! -f "$rviz_dir/dist/index.html" ]]; then
            echo "PowerShell reported a completed Web RViz build, but dist/index.html is missing." >&2
            exit 2
          fi
        elif [[ -n "${3:-}" ]]; then
          echo "Unexpected core build option: $3" >&2
          usage >&2
          exit 2
        elif command -v npm >/dev/null 2>&1 && [[ "$(node -p 'process.platform' 2>/dev/null || true)" == "linux" ]]; then
          npm --prefix "$rviz_dir" ci
          npm --prefix "$rviz_dir" run build
        elif [[ ! -f "$rviz_dir/dist/index.html" ]]; then
          echo "Web RViz build assets are missing. Install Node.js 20+ and npm, then run npm ci and npm run build in src/core/embodied_nexus_core/web_rviz." >&2
          exit 2
        fi
        colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_core
        ;;
      sim)
        colcon build --base-paths src --symlink-install --packages-up-to embodied_nexus_sim_gazebo embodied_nexus_sim_mujoco
        ;;
      *)
        echo "build target must be core or sim" >&2
        usage >&2
        exit 2
        ;;
    esac
    if [[ -f install/setup.bash ]]; then
      source install/setup.bash
    fi
    ;;
  start-mock)
    if [[ -z "${EMBODIED_NEXUS_CORE_TOKEN:-}" ]]; then
      echo "Set EMBODIED_NEXUS_CORE_TOKEN in this Bash environment before starting Core." >&2
      exit 2
    fi
    exec ros2 run embodied_nexus_core nexus_core --config src/core/embodied_nexus_core/config/platform_mock.yaml
    ;;
  status)
    exec ros2 service call /embodied_nexus/core/get_status embodied_nexus_core_interfaces/srv/GetCoreStatus "{}"
    ;;
  device)
    action="${2:-}"
    device_id="${3:-}"
    case "$action" in
      start|restart|shutdown) ;;
      *)
        echo "device action must be start, restart or shutdown" >&2
        usage >&2
        exit 2
        ;;
    esac
    if [[ -z "$device_id" ]]; then
      echo "device_id is required" >&2
      usage >&2
      exit 2
    fi
    yaml_device_id="$(printf '%s' "$device_id" | sed "s/'/''/g")"
    exec ros2 service call /embodied_nexus/core/manage_device embodied_nexus_core_interfaces/srv/ManageCoreDevice \
      "{device_id: '$yaml_device_id', action: $action}"
    ;;
  batch)
    action="${2:-}"
    case "$action" in
      start_all|stop_all|shutdown_all|shutdown_core) ;;
      *)
        echo "batch action must be start_all, stop_all, shutdown_all or shutdown_core" >&2
        usage >&2
        exit 2
        ;;
    esac
    exec ros2 service call /embodied_nexus/core/manage_all embodied_nexus_core_interfaces/srv/ManageAllCoreDevices \
      "{action: $action}"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "Unknown Core command: $command_name" >&2
    usage >&2
    exit 2
    ;;
esac
