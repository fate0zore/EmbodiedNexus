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
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from embodied_nexus_visual_servo_interfaces.action import VisualServoGrasp

rclpy.init()
node = Node('visual_servo_readiness_probe')
client = ActionClient(node, VisualServoGrasp, '/embodied_nexus/visual_servo/grasp')
try:
    ready = (client.wait_for_server(timeout_sec=3.0) and
             node.count_publishers('/embodied_nexus/visual_servo/target') > 0)
finally:
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
# Args: optional --gui and --core. Returns: zero when the stack is ready.
start_stack() {
  local log pid info gui_value=false core_value=false option
  for option in "$@"; do
    case "$option" in
      --gui) [[ "$gui_value" == false ]] || { echo 'Duplicate --gui.' >&2; return 2; }; gui_value=true ;;
      --core) [[ "$core_value" == false ]] || { echo 'Duplicate --core.' >&2; return 2; }; core_value=true ;;
      *) echo "Unknown option: $option" >&2; return 2 ;;
    esac
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
  setsid ros2 launch embodied_nexus_visual_servo visual_servo.launch.py "gui:=$gui_value" \
    "start_core:=$core_value" > "$log" 2>&1 < /dev/null &
  pid=$!
  sleep 0.3
  info="$(identity "$pid")" || { echo "Launch exited; inspect $log" >&2; return 1; }
  printf '%s %s\n' "$pid" "$info" > "$PID_FILE"
  printf '%s\n' "$log" > "$STATE_DIR/launch.logpath"
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
  local mode="$1"
  owned_running || { echo 'This script has not started a running visual servo stack.' >&2; return 1; }
  load_environment
  if [[ "$mode" == verify ]]; then
    ros2 run embodied_nexus_visual_servo visual_servo_acceptance --trials 1 \
      --object-id cylinder-1 --camera-id camera_front --arm-id arm_main \
      --gripper-id gripper_main --target-class 'green cylinder'
  else
    ros2 action send_goal "$ACTION" embodied_nexus_visual_servo_interfaces/action/VisualServoGrasp \
      "{target_class: 'green cylinder', object_id: 'cylinder-1', camera_id: 'camera_front', arm_id: 'arm_main', gripper_id: 'gripper_main'}" \
      --feedback
  fi
}

usage='Usage: bash servo_test.sh [start [--gui] [--core] | restart [--gui] [--core] | grasp | verify | status | stop]'
case "${1:-start}" in
  start|restart)
    command="${1:-start}"
    if [[ $# -gt 0 ]]; then shift; fi
    gui_seen=false; core_seen=false
    for option in "$@"; do
      case "$option" in
        --gui) [[ "$gui_seen" == false ]] || { echo "$usage" >&2; exit 2; }; gui_seen=true ;;
        --core) [[ "$core_seen" == false ]] || { echo "$usage" >&2; exit 2; }; core_seen=true ;;
        *) echo "$usage" >&2; exit 2 ;;
      esac
    done
    [[ "$command" != restart ]] || stop_stack
    start_stack "$@" ;;
  grasp|verify) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; run_grasp "$1" ;;
  stop) [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }; stop_stack ;;
  status)
    [[ $# -eq 1 ]] || { echo "$usage" >&2; exit 2; }
    if owned_running; then echo 'visual servo: running'; else echo 'visual servo: stopped'; fi
    if [[ -f "$STATE_DIR/launch.logpath" ]]; then echo "Log: $(cat "$STATE_DIR/launch.logpath")"; fi ;;
  -h|--help|help) echo "$usage" ;;
  *) echo "$usage" >&2; exit 2 ;;
esac
