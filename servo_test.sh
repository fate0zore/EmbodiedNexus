#!/usr/bin/env bash
set -eo pipefail

# Local WSL MuJoCo visual servo demo. Starting does not trigger robot motion.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BASE_SETUP="${ENX_SERVO_BASE_SETUP:-$HOME/.cache/embodied-nexus/grasp-hardness-install/setup.bash}"
if [[ -n "${ENX_SERVO_SETUP:-}" ]]; then
  SETUP="$ENX_SERVO_SETUP"
elif [[ -f "$HOME/.cache/embodied-nexus/vs2-install/local_setup.bash" ]]; then
  SETUP="$HOME/.cache/embodied-nexus/vs2-install/local_setup.bash"
else
  SETUP="$ROOT/install/setup.bash"
fi
RUNTIME="${ENX_SERVO_RUNTIME:-/mnt/d/enx-local-cache/operator-runtime-py310}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/embodied-nexus/servo-test"
PID_FILE="$STATE_DIR/launch.pid"
ACTION="/embodied_nexus/visual_servo/grasp"
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
from embodied_nexus_visual_servo_interfaces.action import VisualServoGrasp
from std_srvs.srv import Trigger
from embodied_nexus_arm_interfaces.srv import GetArmState
from embodied_nexus_gripper_interfaces.srv import GetGripperState

rclpy.init()
node = Node('visual_servo_readiness_probe')
client = ActionClient(node, VisualServoGrasp, '/embodied_nexus/visual_servo/grasp')
tracker = node.create_client(Trigger, '/embodied_nexus/visual_servo/reset_tracker')
arm = node.create_client(GetArmState, '/embodied_nexus/arms/arm_main/get_state')
gripper = node.create_client(GetGripperState, '/embodied_nexus/grippers/gripper_main/get_state')
try:
    ready = (client.wait_for_server(timeout_sec=3.0) and
             node.count_publishers('/embodied_nexus/visual_servo/target') > 0 and
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

# Load Humble, the optional local underlay, and the visual servo overlay.
# Args: none. Returns: zero when the requested setup is available.
load_environment() {
  [[ -f /opt/ros/humble/setup.bash && -f "$SETUP" ]] || {
    echo "Missing ROS Humble or visual servo overlay: $SETUP" >&2; return 1; }
  source /opt/ros/humble/setup.bash
  if [[ -f "$BASE_SETUP" && "$BASE_SETUP" != "$SETUP" ]]; then
    source "$BASE_SETUP"
  fi
  source "$SETUP"
  if [[ -d "$RUNTIME" ]]; then
    export PYTHONPATH="$RUNTIME${PYTHONPATH:+:$PYTHONPATH}"
  fi
  export YOLO_AUTOINSTALL=False
  export MUJOCO_GL="${MUJOCO_GL:-egl}"
  export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
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
  echo 'Visual servo test processes stopped.'
}

# Start one launch process and wait for the Action and perception publisher.
# Args: optional GUI, Core, scene, class and object settings. Returns: zero when ready.
start_stack() {
  local log pid info gui_value=false core_value=false scene_value=visual_servo_contact
  local class_value='green cylinder' object_value=cylinder-1 scene_dir=''
  local -a launch_args
  while (($#)); do
    case "$1" in
      --gui) [[ "$gui_value" == false ]] || { echo 'Duplicate --gui.' >&2; return 2; }; gui_value=true ;;
      --core) [[ "$core_value" == false ]] || { echo 'Duplicate --core.' >&2; return 2; }; core_value=true ;;
      --scene|--target-class|--object-id|--scene-directory)
        [[ $# -ge 2 && -n "$2" && "$2" != *$'\n'* ]] || { echo "Missing or invalid value for $1" >&2; return 2; }
        case "$1" in
          --scene) scene_value="$2" ;;
          --target-class) class_value="$2" ;;
          --object-id) object_value="$2" ;;
          --scene-directory) scene_dir="$2" ;;
        esac
        shift ;;
      *) echo "Unknown option: $1" >&2; return 2 ;;
    esac
    shift
  done
  if owned_running; then
    echo 'Visual servo test is already running; use status or stop.' >&2; return 1
  fi
  if pgrep -f '/moveit_ros_move_group/move_group' > /dev/null; then
    echo 'A MoveIt move_group process is already running; stop the existing session first.' >&2; return 1
  fi
  if [[ "$core_value" == true ]] && { port_open 8780 || port_open 9091; }; then
    echo 'Core port 8780 or rosbridge port 9091 is already in use.' >&2; return 1
  fi
  load_environment
  if ! python3 -c 'import torch, ultralytics; from embodied_nexus_sim.adapters.aubo_i5.backend import Ros2SimArmBackend' >/dev/null; then
    echo 'Visual servo Python dependencies or simulation adapters are unavailable.' >&2; return 1
  fi
  log="$STATE_DIR/launch-$(date +%Y%m%d-%H%M%S)-$$.log"
  launch_args=("gui:=$gui_value" "start_core:=$core_value" "scene:=$scene_value" "target_class:=$class_value")
  [[ -z "$scene_dir" ]] || launch_args+=("scene_directory:=$scene_dir")
  setsid ros2 launch embodied_nexus_visual_servo visual_servo.launch.py "${launch_args[@]}" > "$log" 2>&1 < /dev/null &
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
      echo 'Visual servo is ready; no grasp has been triggered.'
      if [[ "$core_value" == true ]]; then
        echo 'Core: http://127.0.0.1:8780/'
        echo 'Web RViz: http://127.0.0.1:8780/rviz/'
      fi
      echo "Trigger a grasp: bash $ROOT/servo_test.sh grasp"
      echo "Run independent verification: bash $ROOT/servo_test.sh verify"
      echo "Log: $log"
      return 0
    fi
    sleep 1
  done
  echo "Startup did not become ready; inspect $log" >&2
  stop_stack || echo 'Cleanup did not complete; inspect the retained process record.' >&2
  return 1
}

# Trigger one visual servo Action or the independent truth-based acceptance run.
# Args: grasp or verify. Returns: command exit status.
run_grasp() {
  local mode="$1" goal
  local -a selection
  owned_running || { echo 'This script has not started a running visual servo stack.' >&2; return 1; }
  load_environment
  mapfile -t selection < "$STATE_DIR/selection"
  if [[ "$mode" == verify ]]; then
    ros2 run embodied_nexus_visual_servo visual_servo_acceptance --trials 1 \
      --scene-id "${selection[0]}" --object-id "${selection[2]}" --camera-id camera_front --arm-id arm_main \
      --gripper-id gripper_main --target-class "${selection[1]}"
  else
    goal="$(python3 -c 'import json,sys; print(json.dumps(dict(target_class=sys.argv[1], object_id=sys.argv[2], camera_id="camera_front", arm_id="arm_main", gripper_id="gripper_main")))' "${selection[1]}" "${selection[2]}")"
    ros2 action send_goal "$ACTION" embodied_nexus_visual_servo_interfaces/action/VisualServoGrasp \
      "$goal" \
      --feedback
  fi
}

usage='Usage: bash servo_test.sh [start|restart] [--gui] [--core] [--scene ID] [--scene-directory DIR] [--target-class PROMPT] [--object-id ID] | grasp | verify | status | stop'
case "${1:-start}" in
  start|restart)
    command="${1:-start}"
    if [[ $# -gt 0 ]]; then shift; fi
    options=("$@")
    while (($#)); do
      case "$1" in
        --gui|--core) ;;
        --scene|--target-class|--object-id|--scene-directory)
          [[ $# -ge 2 && -n "$2" && "$2" != *$'\n'* ]] || { echo "$usage" >&2; exit 2; }
          shift ;;
        *) echo "$usage" >&2; exit 2 ;;
      esac
      shift
    done
    [[ "$command" != restart ]] || stop_stack
    start_stack "${options[@]}" ;;
  grasp|verify) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; run_grasp "$1" ;;
  stop) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; stop_stack ;;
  status)
    [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }
    if owned_running; then echo 'visual servo: running'; else echo 'visual servo: stopped'; fi
    if [[ -f "$STATE_DIR/launch.logpath" ]]; then echo "Log: $(cat "$STATE_DIR/launch.logpath")"; fi ;;
  -h|--help|help) echo "$usage" ;;
  *) echo "$usage" >&2; exit 2 ;;
esac
