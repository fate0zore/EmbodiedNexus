#!/usr/bin/env bash
set -eo pipefail

# Local WSL MuJoCo dynamic grasp demo. Starting does not trigger robot motion.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE_SETUP="${ENX_DYNAMIC_BASE_SETUP:-}"
if [[ -n "${ENX_DYNAMIC_SETUP:-}" ]]; then
  SETUP="$ENX_DYNAMIC_SETUP"
else
  SETUP="${XDG_CACHE_HOME:-$HOME/.cache}/embodied-nexus/dynamic-grasp/install/setup.bash"
fi
RUNTIME="${ENX_DYNAMIC_RUNTIME:-/mnt/d/enx-local-cache/operator-runtime-py310}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/embodied-nexus/dynamic-grasp-test"
PID_FILE="$STATE_DIR/launch.pid"
ACTION="/embodied_nexus/dynamic_grasp/grasp"
mkdir -p "$STATE_DIR"

# Print a process start time and boot ID; fail for absent or zombie processes.
# Args: PID. Returns: identity on stdout, or a nonzero status.
identity() {
  python3 - "$1" <<'PY'
import pathlib, sys
try:
    text = pathlib.Path('/proc', sys.argv[1], 'stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    if fields[0] == 'Z':
        raise ProcessLookupError()
    print(fields[19], pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip())
except (OSError, ProcessLookupError):
    sys.exit(1)
PY
}

# Verify the recorded PID still identifies the process started by this script.
# Args: none. Returns: zero only for the same live process.
owned_running() {
  local pid start boot actual
  [[ -f "$PID_FILE" ]] || return 1
  read -r pid start boot < "$PID_FILE"
  actual="$(identity "$pid" 2>/dev/null)" || return 1
  [[ "$actual" == "$start $boot" ]]
}

# Check whether a process group still contains a non-zombie process.
# Args: process group ID. Returns: zero if a live member exists.
group_running() {
  ps -eo pgid=,stat= 2>/dev/null | awk -v group="$1" '$1 == group && $2 !~ /^Z/ { found = 1 } END { exit !found }'
}

# Check whether a local TCP port has an active listener.
# Args: port number. Returns: zero if a connection succeeds.
port_open() {
  python3 - "$1" <<'PY'
import socket, sys
with socket.socket() as sock:
    sock.settimeout(0.3)
    sys.exit(0 if sock.connect_ex(('127.0.0.1', int(sys.argv[1]))) == 0 else 1)
PY
}

# Probe discovery directly so startup does not depend on the ROS CLI daemon.
# Args: none. Returns: zero when both the Action and perception publisher exist.
stack_ready() {
  timeout 8s python3 - <<'PY'
import sys
import json
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from embodied_nexus_dynamic_grasp_interfaces.action import DynamicGrasp
from std_srvs.srv import Trigger
from std_msgs.msg import String
from embodied_nexus_arm_interfaces.srv import GetArmState
from embodied_nexus_gripper_interfaces.srv import GetGripperState

rclpy.init()
node = Node('dynamic_grasp_readiness_probe')
client = ActionClient(node, DynamicGrasp, '/embodied_nexus/dynamic_grasp/grasp')
tracker = node.create_client(Trigger, '/embodied_nexus/dynamic_grasp/reset_tracker')
fixture_prepare = node.create_publisher(String, '/embodied_nexus/dynamic_grasp/fixture_prepare', 10)
fixture_start = node.create_publisher(String, '/embodied_nexus/dynamic_grasp/fixture_start', 10)
arm = node.create_client(GetArmState, '/embodied_nexus/arms/arm_main/get_state')
gripper = node.create_client(GetGripperState, '/embodied_nexus/grippers/gripper_main/get_state')
try:
    ready = (client.wait_for_server(timeout_sec=3.0) and
             node.count_publishers('/embodied_nexus/dynamic_grasp/target') > 0 and
             node.count_publishers('/embodied_nexus/dynamic_grasp/fixture_event') > 0 and
             fixture_prepare.get_subscription_count() > 0 and fixture_start.get_subscription_count() > 0 and
             tracker.wait_for_service(timeout_sec=3.0) and
             arm.wait_for_service(timeout_sec=1.0) and gripper.wait_for_service(timeout_sec=1.0))
    if ready:
        arm_future = arm.call_async(GetArmState.Request())
        gripper_future = gripper.call_async(GetGripperState.Request())
        rclpy.spin_until_future_complete(node, arm_future, timeout_sec=1.0)
        rclpy.spin_until_future_complete(node, gripper_future, timeout_sec=1.0)
        ready = (arm_future.done() and gripper_future.done() and arm_future.result() is not None and
                 gripper_future.result() is not None and
                 json.loads(arm_future.result().state_json).get('feedback_valid', False) and
                 gripper_future.result().state.feedback_valid and gripper_future.result().state.connected)
finally:
    node.destroy_client(arm)
    node.destroy_client(gripper)
    node.destroy_client(tracker)
    client.destroy()
    node.destroy_node()
    rclpy.shutdown()
sys.exit(0 if ready else 1)
PY
}

# Load Humble, the optional local underlay, and the dynamic grasp overlay.
# Args: none. Returns: zero when the requested setup is available.
load_environment() {
  [[ -f /opt/ros/humble/setup.bash && -f "$SETUP" ]] || {
    echo "Missing ROS Humble or dynamic grasp overlay: $SETUP" >&2
    echo "Build the dynamic grasp workspace as described in src/operator/docs/dynamic-grasp.md." >&2
    return 1; }
  source /opt/ros/humble/setup.bash
  if [[ -n "$BASE_SETUP" && ! -f "$BASE_SETUP" ]]; then
    echo "Missing requested underlay: $BASE_SETUP" >&2; return 1
  fi
  if [[ -n "$BASE_SETUP" && "$BASE_SETUP" != "$SETUP" ]]; then
    source "$BASE_SETUP"
  fi
  source "$SETUP"
  if [[ -d "$RUNTIME" ]]; then
    export PYTHONPATH="$RUNTIME${PYTHONPATH:+:$PYTHONPATH}"
  fi
  export YOLO_AUTOINSTALL=False
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
  if ! python3 -c 'from embodied_nexus_dynamic_grasp_interfaces.action import DynamicGrasp; import embodied_nexus_dynamic_grasp.demo' >/dev/null; then
    echo "Dynamic grasp package or generated interfaces are missing from $SETUP." >&2
    echo "Rebuild the dynamic grasp workspace; see src/operator/docs/dynamic-grasp.md." >&2
    return 1
  fi
  if ! ros2 pkg prefix embodied_nexus_dynamic_grasp >/dev/null 2>&1; then
    echo "Dynamic grasp ROS package is missing from $SETUP; rebuild the workspace." >&2
    return 1
  fi
}

# Stop only the verified process group created by this script.
# Args: none. Returns: zero after shutdown or if it was not running.
stop_stack() {
  local pid start boot pgid
  if owned_running; then
    read -r pid start boot < "$PID_FILE"
    pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
    if [[ "$pgid" != "$pid" ]]; then
      echo "Skipping PID $pid: process group changed." >&2; return 1
    fi
    kill -INT -- "-$pid" 2>/dev/null || true
    for _ in {1..40}; do
      group_running "$pid" || break
      sleep 0.25
    done
    if group_running "$pid"; then
      kill -TERM -- "-$pid" 2>/dev/null || true
      for _ in {1..40}; do
        group_running "$pid" || break
        sleep 0.25
      done
    fi
    if group_running "$pid"; then
      echo "Process group $pid did not stop; retaining $PID_FILE." >&2; return 1
    fi
  fi
  rm -f -- "$PID_FILE"
  echo 'Dynamic grasp test processes stopped.'
}

# Start one launch process and wait for the Action and perception publisher.
# Args: optional GUI, Core, scene, class and object settings. Returns: zero when ready.
start_stack() {
  local log pid info gui_value=false core_value=false scene_value=dynamic_grasp_tabletop_cylinder
  local class_value='green cylinder' object_value=cylinder-1 scene_dir='' mass_value=0.1
  local -a launch_args
  while (($#)); do
    case "$1" in
      --gui) [[ "$gui_value" == false ]] || { echo 'Duplicate --gui.' >&2; return 2; }; gui_value=true ;;
      --core) [[ "$core_value" == false ]] || { echo 'Duplicate --core.' >&2; return 2; }; core_value=true ;;
      --scene|--target-class|--object-id|--scene-directory|--object-mass-kg)
        [[ $# -ge 2 && -n "$2" && "$2" != *$'\n'* ]] || { echo "Missing or invalid value for $1" >&2; return 2; }
        case "$1" in
          --scene) scene_value="$2" ;;
          --target-class) class_value="$2" ;;
          --object-id) object_value="$2" ;;
          --scene-directory) scene_dir="$2" ;;
          --object-mass-kg) mass_value="$2" ;;
        esac
        shift ;;
      *) echo "Unknown option: $1" >&2; return 2 ;;
    esac
    shift
  done
  if owned_running; then
    echo 'Dynamic grasp test is already running; use status or stop.' >&2; return 1
  fi
  if pgrep -f '/moveit_ros_move_group/move_group' > /dev/null; then
    echo 'A MoveIt move_group process is already running; stop the existing session first.' >&2; return 1
  fi
  if [[ "$core_value" == true ]] && { port_open 8780 || port_open 9091; }; then
    echo 'Core port 8780 or rosbridge port 9091 is already in use.' >&2; return 1
  fi
  load_environment
  if ! python3 -c 'import torch, ultralytics; from embodied_nexus_sim.adapters.aubo_i5.backend import Ros2SimArmBackend' >/dev/null; then
    echo 'Dynamic grasp Python dependencies or simulation adapters are unavailable.' >&2; return 1
  fi
  log="$STATE_DIR/launch-$(date +%Y%m%d-%H%M%S)-$$.log"
  launch_args=("gui:=$gui_value" "start_core:=$core_value" "scene:=$scene_value" "target_class:=$class_value" "object_id:=$object_value" "object_mass_kg:=$mass_value" "collision_diagnostics:=${ENX_DYNAMIC_COLLISION_DIAGNOSTICS:-false}")
  [[ -z "$scene_dir" ]] || launch_args+=("scene_directory:=$scene_dir")
  setsid ros2 launch embodied_nexus_dynamic_grasp dynamic_grasp.launch.py "${launch_args[@]}" > "$log" 2>&1 < /dev/null &
  pid=$!
  sleep 0.3
  info="$(identity "$pid")" || { echo "Launch exited; inspect $log" >&2; return 1; }
  printf '%s %s\n' "$pid" "$info" > "$PID_FILE"
  printf '%s\n' "$log" > "$STATE_DIR/launch.logpath"
  printf '%s\n%s\n%s\n' "$scene_value" "$class_value" "$object_value" > "$STATE_DIR/selection"
  echo "Using ROS overlay: $SETUP"
  for _ in {1..90}; do
    if ! owned_running || grep -q 'process has died' "$log"; then break; fi
    if stack_ready && { [[ "$core_value" == false ]] || { port_open 8780 && port_open 9091; }; }; then
      echo 'Dynamic grasp is ready; no grasp has been triggered.'
      if [[ "$core_value" == true ]]; then
        echo 'Core: http://127.0.0.1:8780/'
        echo 'Web RViz: http://127.0.0.1:8780/rviz/'
      fi
      echo "Trigger a grasp: bash $ROOT/dynamic_grasp_test.sh grasp"
      echo "Log: $log"
      return 0
    fi
    sleep 1
  done
  echo "Startup did not become ready; inspect $log" >&2
  stop_stack || echo 'Cleanup did not complete; inspect the retained process record.' >&2
  return 1
}

# Trigger the fixture and the independent dynamic grasp Action.
# Args: none. Returns: the demonstration exit status.
run_grasp() {
  local -a selection
  owned_running || { echo 'This script has not started a running dynamic grasp stack.' >&2; return 1; }
  load_environment
  [[ -f "$STATE_DIR/selection" ]] || { echo 'Missing saved scene selection.' >&2; return 1; }
  mapfile -t selection < "$STATE_DIR/selection"
  [[ ${#selection[@]} -eq 3 ]] || { echo 'Invalid saved scene selection.' >&2; return 1; }
  ros2 run embodied_nexus_dynamic_grasp dynamic_grasp_demo \
    --target-class "${selection[1]}" --object-id "${selection[2]}"
}

usage='Usage: bash dynamic_grasp_test.sh [start|restart] [--gui] [--core] [--scene ID] [--scene-directory DIR] [--target-class PROMPT] [--object-id ID] [--object-mass-kg KG] | grasp | status | stop'
case "${1:-start}" in
  start|restart)
    command="${1:-start}"
    if [[ $# -gt 0 ]]; then shift; fi
    options=("$@")
    while (($#)); do
      case "$1" in
        --gui|--core) ;;
        --scene|--target-class|--object-id|--scene-directory|--object-mass-kg)
          [[ $# -ge 2 && -n "$2" && "$2" != *$'\n'* ]] || { echo "$usage" >&2; exit 2; }
          shift ;;
        *) echo "$usage" >&2; exit 2 ;;
      esac
      shift
    done
    [[ "$command" != restart ]] || stop_stack
    start_stack "${options[@]}" ;;
  grasp) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; run_grasp ;;
  stop) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; stop_stack ;;
  status)
    [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }
    if owned_running; then echo 'dynamic grasp: running'; else echo 'dynamic grasp: stopped'; fi
    if [[ -f "$STATE_DIR/launch.logpath" ]]; then echo "Log: $(cat "$STATE_DIR/launch.logpath")"; fi ;;
  -h|--help|help) echo "$usage" ;;
  *) echo "$usage" >&2; exit 2 ;;
esac
