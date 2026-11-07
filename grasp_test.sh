#!/usr/bin/env bash
set -eo pipefail

# Local WSL MuJoCo grasp demo. Never downloads dependencies or triggers a grasp on start.
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -n "${ENX_GRASP_SETUP:-}" ]]; then
  SETUP="$ENX_GRASP_SETUP"
elif [[ -f "$HOME/.cache/embodied-nexus/grasp-hardness-install/setup.bash" ]]; then
  SETUP="$HOME/.cache/embodied-nexus/grasp-hardness-install/setup.bash"
else
  SETUP="$HOME/.cache/embodied-nexus/operator-install/setup.bash"
fi
RUNTIME="${ENX_GRASP_RUNTIME:-/mnt/d/enx-local-cache/operator-runtime-py310}"
TOKEN_FILE="${ENX_GRASP_TOKEN_FILE:-$HOME/.cache/embodied-nexus/operator-http.token}"
STATE_DIR="${XDG_STATE_HOME:-$HOME/.local/state}/embodied-nexus/grasp-test"
SERVICES_STATE="${XDG_STATE_HOME:-$HOME/.local/state}/embodied-nexus/services.json"
SCENE="scene-1793463913737"
mkdir -p "$STATE_DIR"

# Return the process identity used to avoid stopping a reused PID.
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

# Check a PID and start time written by this script.
owned_running() {
  local file="$1" pid start boot actual
  [[ -f "$file" ]] || return 1
  read -r pid start boot < "$file"
  actual="$(identity "$pid" 2>/dev/null)" || return 1
  [[ "$actual" == "$start $boot" ]]
}

# Check whether any non-zombie process remains in the verified process group.
group_running() {
  ps -eo pgid=,stat= 2>/dev/null | awk -v group="$1" '$1 == group && $2 !~ /^Z/ { found = 1 } END { exit !found }'
}

# Start one independent ROS launch process and record its identity.
start_process() {
  local name="$1"; shift
  local log="$STATE_DIR/$name-$(date +%Y%m%d-%H%M%S)-$$.log"
  setsid "$@" > "$log" 2>&1 < /dev/null &
  local pid=$! info
  sleep 0.3
  info="$(identity "$pid")" || { echo "$name exited; inspect $log" >&2; return 1; }
  printf '%s %s\n' "$pid" "$info" > "$STATE_DIR/$name.pid"
  printf '%s\n' "$log" > "$STATE_DIR/$name.logpath"
}

# Stop only an intact process group started by this script.
stop_process() {
  local file="$1" pid start boot pgid
  if owned_running "$file"; then
    read -r pid start boot < "$file"
    pgid="$(ps -o pgid= -p "$pid" | tr -d ' ')"
    if [[ "$pgid" == "$pid" ]]; then
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
        echo "Process group $pid did not stop; retaining $file" >&2
        return 1
      fi
    else
      echo "Skipping PID $pid: process group changed" >&2
      return 1
    fi
  fi
  rm -f -- "$file"
}

# Read the managed simulator identity recorded by the existing services tool.
managed_identity() {
  python3 - "$SERVICES_STATE" <<'PY'
import json, pathlib, sys
try:
    item = json.loads(pathlib.Path(sys.argv[1]).read_text())['sim:mujoco:managed']['processes']['system']
    print(item['pid'], item['start'], item['boot_id'])
except (OSError, ValueError, KeyError, TypeError):
    sys.exit(1)
PY
}

# Check whether a local TCP listener is active.
port_open() {
  python3 - "$1" <<'PY'
import socket, sys
with socket.socket() as sock:
    sock.settimeout(0.3)
    sys.exit(0 if sock.connect_ex(('127.0.0.1', int(sys.argv[1]))) == 0 else 1)
PY
}

# Stop the operator, MoveIt and the managed simulator owned by this script.
stop_stack() {
  stop_process "$STATE_DIR/operator.pid" || return 1
  stop_process "$STATE_DIR/moveit.pid" || return 1
  local recorded current sim_pid core_pid bridge_pid bridge_info child_pid child_args child_info
  if [[ -f "$STATE_DIR/managed.pid" ]]; then
    recorded="$(cat "$STATE_DIR/managed.pid")"
    current="$(managed_identity 2>/dev/null || true)"
    if [[ -n "$current" && "$current" == "$recorded" ]] && owned_running "$STATE_DIR/managed.pid"; then
      read -r sim_pid _ < "$STATE_DIR/managed.pid"
      core_pid="$(pgrep -P "$sim_pid" -f '/nexus_core' | head -n 1 || true)"
      bridge_pid=""
      if [[ -n "$core_pid" ]]; then
        bridge_pid="$(pgrep -P "$core_pid" -f 'ros2 launch rosbridge_server' | head -n 1 || true)"
      fi
      if [[ -n "$bridge_pid" ]]; then
        bridge_info="$(identity "$bridge_pid" 2>/dev/null || true)"
        [[ -z "$bridge_info" ]] || printf '%s %s\n' "$bridge_pid" "$bridge_info" > "$STATE_DIR/bridge.pid"
      fi
      if ! bash "$ROOT/scripts/services.sh" stop --mode sim --simulator mujoco --scene "$SCENE" -all --setup "$SETUP"; then
        echo 'Core could not confirm device shutdown; stopping owned local simulation process groups.' >&2
        if [[ -z "$core_pid" ]] || ! owned_running "$STATE_DIR/managed.pid" ||
           [[ "$(managed_identity 2>/dev/null || true)" != "$recorded" ]]; then
          echo 'Cannot confirm the owned MuJoCo/Core session; leaving it running.' >&2
          return 1
        fi
        while read -r child_pid; do
          [[ -n "$child_pid" ]] || continue
          child_args="$(ps -o args= -p "$child_pid" || true)"
          if [[ "$child_args" == *'ros2 launch rosbridge_server '* ]] ||
             { [[ "$child_args" == *'ros2 launch embodied_nexus_'* ]] &&
               [[ "$child_args" == *'config_file:='*'_sim.yaml'* ]]; }; then
            child_info="$(identity "$child_pid" 2>/dev/null || true)"
            if [[ -n "$child_info" ]]; then
              printf '%s %s\n' "$child_pid" "$child_info" > "$STATE_DIR/fallback-$child_pid.pid"
              stop_process "$STATE_DIR/fallback-$child_pid.pid" || return 1
            fi
          else
            echo "Unrecognized Core child $child_pid; leaving the session running." >&2
            return 1
          fi
        done < <(pgrep -P "$core_pid" || true)
        stop_process "$STATE_DIR/managed.pid" || return 1
      fi
      stop_process "$STATE_DIR/bridge.pid" || return 1
    elif [[ -n "$current" && "$current" != "$recorded" ]]; then
      echo 'Managed simulator identity changed; leaving the other session running.' >&2
      return 1
    fi
    rm -f -- "$STATE_DIR/managed.pid"
  fi
  echo 'Grasp test processes stopped.'
}

# Start the local stack and wait until the operator HTTP port and MoveIt are ready.
start_stack() {
  local gui="${1:-}"
  for file in "$STATE_DIR/operator.pid" "$STATE_DIR/moveit.pid" "$STATE_DIR/managed.pid"; do
    if owned_running "$file"; then echo 'Grasp test is already running; use status or stop.' >&2; return 1; fi
  done
  if port_open 8780 || port_open 8782; then
    echo 'Port 8780 or 8782 is already in use; stop the existing session first.' >&2; return 1
  fi
  if pgrep -f '/moveit_ros_move_group/move_group' > /dev/null; then
    echo 'A MoveIt move_group process is already running; stop the existing session first.' >&2; return 1
  fi
  [[ -f "$SETUP" && -d "$RUNTIME" && -s "$TOKEN_FILE" ]] || {
    echo 'Local ROS overlay, Python runtime or operator token is missing.' >&2; return 1; }
  # ROS setup scripts are not compatible with Bash nounset.
  source /opt/ros/humble/setup.bash
  source "$SETUP"
  export PYTHONPATH="$RUNTIME${PYTHONPATH:+:$PYTHONPATH}"
  export YOLO_AUTOINSTALL=False
  export EMBODIED_NEXUS_OPERATOR_HTTP_TOKEN="$(cat "$TOKEN_FILE")"
  if ! python3 -c 'import embodied_nexus_sim.adapters.scout_2.backend' >/dev/null; then
    echo "ROS overlay is incomplete: $SETUP (embodied_nexus_sim adapters could not be imported)." >&2
    return 1
  fi
  echo "Using ROS overlay: $SETUP"
  local args=(start --mode sim --simulator mujoco --scene "$SCENE" -all --setup "$SETUP")
  [[ "$gui" != '--gui' ]] || args+=(--gui)
  local previous current
  previous="$(managed_identity 2>/dev/null || true)"
  if ! bash "$ROOT/scripts/services.sh" "${args[@]}"; then
    current="$(managed_identity 2>/dev/null || true)"
    if [[ -n "$current" && "$current" != "$previous" ]]; then
      printf '%s\n' "$current" > "$STATE_DIR/managed.pid"
      echo 'Startup failed; stopping the simulator session created by this attempt.' >&2
      stop_stack || echo 'Cleanup did not complete; inspect the retained process records.' >&2
    fi
    return 1
  fi
  managed_identity > "$STATE_DIR/managed.pid"
  if ! start_process moveit ros2 launch embodied_nexus_operator_moveit_config move_group.launch.py use_sim_time:=true soft_finger_pads:=true ||
     ! start_process operator ros2 launch embodied_nexus_operator operator.launch.py; then
    stop_stack; return 1
  fi
  for _ in {1..60}; do
    if ! owned_running "$STATE_DIR/moveit.pid" || ! owned_running "$STATE_DIR/operator.pid"; then break; fi
    if port_open 8782 && grep -q 'You can start planning now' "$(cat "$STATE_DIR/moveit.logpath")"; then
      echo 'Ready: Web RViz http://127.0.0.1:8780/rviz/'
      echo "Trigger a grasp: bash $ROOT/grasp_test.sh grasp"
      echo "Logs: $STATE_DIR"
      return 0
    fi
    sleep 1
  done
  echo "Startup did not become ready; inspect logs in $STATE_DIR" >&2
  stop_stack
  return 1
}

# Trigger exactly one ROS action and stream its feedback.
grasp() {
  [[ -f "$SETUP" ]] || { echo "Missing ROS overlay: $SETUP" >&2; return 1; }
  source /opt/ros/humble/setup.bash
  source "$SETUP"
  owned_running "$STATE_DIR/operator.pid" && port_open 8782 || {
    echo 'This script has not started a running grasp operator.' >&2; return 1; }
  ros2 action send_goal /embodied_nexus/operators/grasp \
    embodied_nexus_operator_interfaces/action/PlanGrasp \
    "{target_class: 'green cylinder', object_id: 'cylinder-1', camera_id: 'camera_front', arm_id: 'arm_main', gripper_id: 'gripper_main'}" \
    --feedback
}

case "${1:-start}" in
  start) [[ $# -le 2 && ( $# -lt 2 || "$2" == '--gui' ) ]] || { echo 'Usage: grasp_test.sh start [--gui]' >&2; exit 2; }; start_stack "${2:-}" ;;
  restart) [[ $# -le 2 && ( $# -lt 2 || "$2" == '--gui' ) ]] || { echo 'Usage: grasp_test.sh restart [--gui]' >&2; exit 2; }; stop_stack; start_stack "${2:-}" ;;
  grasp) [[ $# -eq 1 ]] || exit 2; grasp ;;
  stop) [[ $# -eq 1 ]] || exit 2; stop_stack ;;
  status)
    [[ $# -eq 1 ]] || exit 2
    for name in managed moveit operator; do
      if owned_running "$STATE_DIR/$name.pid"; then
        echo "$name: running"
      elif [[ "$name" == managed ]] && current="$(managed_identity 2>/dev/null)" &&
           read -r sim_pid sim_start sim_boot <<< "$current" &&
           [[ "$(identity "$sim_pid" 2>/dev/null)" == "$sim_start $sim_boot" ]]; then
        echo 'managed: running (recorded by services, not tracked by grasp_test)'
      else
        echo "$name: stopped"
      fi
    done ;;
  -h|--help|help) echo 'Usage: bash grasp_test.sh [start [--gui] | restart [--gui] | grasp | status | stop]' ;;
  *) echo 'Usage: bash grasp_test.sh [start [--gui] | restart [--gui] | grasp | status | stop]' >&2; exit 2 ;;
esac
