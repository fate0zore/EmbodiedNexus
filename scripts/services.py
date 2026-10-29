#!/usr/bin/env python3
"""Manage local ROS launch processes and the public Core HTTP interface."""

import argparse
import contextlib
import fcntl
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import quote
from urllib.request import Request, urlopen

import yaml

ROOT = Path(__file__).resolve().parent.parent
STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local/state")) / "embodied-nexus"
STATE = STATE_DIR / "services.json"
TYPES = {"base": ("bases", "Base"), "arm": ("arms", "Arm"),
         "gripper": ("grippers", "Gripper"), "camera": ("cameras", "Camera"),
         "lidar": ("lidars", "Lidar")}


def arguments(argv):
    """Parse command-line arguments.

    Args:
        argv: Command-line arguments excluding the executable.

    Returns:
        Parsed arguments.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("start", "stop", "status"))
    parser.add_argument("devices", nargs="*")
    parser.add_argument("--mode", choices=("mock", "real", "sim"), required=True)
    parser.add_argument("--simulator")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--managed", action="store_true")
    group.add_argument("--standalone", action="store_true")
    parser.add_argument("-all", action="store_true")
    parser.add_argument("--confirm-ready", action="store_true")
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--setup", help="Path to the installed workspace setup.bash")
    parser.add_argument("--config", help="Platform YAML for mock or real")
    args = parser.parse_intermixed_args(argv)
    if args.mode == "sim" and not args.simulator:
        parser.error("--simulator is required for sim mode")
    if args.mode != "sim" and args.simulator:
        parser.error("--simulator is only valid in sim mode")
    if args.mode == "sim" and args.simulator != "gazebo":
        parser.error(f"simulator {args.simulator!r} is not implemented")
    if args.mode == "sim" and args.config:
        parser.error("--config is not supported for Gazebo")
    if args.action == "start" and args.mode == "real" and not args.confirm_ready:
        parser.error("real startup requires --confirm-ready")
    if args.all and args.devices:
        parser.error("-all cannot be combined with device names")
    if args.action != "status" and not args.all and not args.devices:
        parser.error("select device names or -all")
    if len(args.devices) != len(set(args.devices)):
        parser.error("duplicate device names")
    return args


def read_state():
    """Read recorded process ownership.

    Args:
        None.

    Returns:
        Dictionary containing active sessions.
    """
    if not STATE.exists():
        return {}
    return json.loads(STATE.read_text(encoding="utf-8"))


def save_state(state):
    """Atomically save non-secret process metadata.

    Args:
        state: Session dictionary to save.

    Returns:
        None.
    """
    temporary = STATE.with_suffix(".tmp")
    temporary.write_text(json.dumps(state, indent=2), encoding="utf-8")
    os.chmod(temporary, 0o600)
    os.replace(temporary, STATE)


def identity(pid):
    """Get a Linux process start-time identity.

    Args:
        pid: Process ID.

    Returns:
        Process start time string, or None when it no longer exists.
    """
    try:
        stat = Path(f"/proc/{pid}/stat").read_text()
        fields = stat[stat.rfind(")") + 2:].split()
        return None if fields[0] == "Z" else fields[19]
    except (FileNotFoundError, ProcessLookupError):
        return None


def alive(record):
    """Check whether the recorded process is still the same process.

    Args:
        record: Process metadata.

    Returns:
        True if its PID and start time match.
    """
    if not record:
        return False
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    return record.get("boot_id") == boot and identity(record["pid"]) == record["start"]


def launch(command, name, env):
    """Start an owned process group with a private log file.

    Args:
        command: Command argument list.
        name: Log file name.
        env: Child environment.

    Returns:
        Non-secret process identity and log path.
    """
    log = STATE_DIR / f"{name}.log"
    with log.open("ab") as output:
        os.chmod(log, 0o600)
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output,
                                   stderr=subprocess.STDOUT, env=env, start_new_session=True)
    start = identity(process.pid)
    if start is None:
        raise RuntimeError(f"{name} exited immediately; inspect {log}")
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    return {"pid": process.pid, "start": start, "boot_id": boot, "log": str(log), "command": command}


def terminate(record):
    """Gracefully terminate a process group that this tool owns.

    Args:
        record: Process metadata including PID and start time.

    Returns:
        None; raises when the process does not exit safely.
    """
    if not owned_group_exists(record):
        return
    for sig, delay in ((signal.SIGINT, 12), (signal.SIGTERM, 8)):
        if not owned_group_exists(record):
            break
        with contextlib.suppress(ProcessLookupError):
            os.killpg(record["pid"], sig)
        deadline = time.monotonic() + delay
        while owned_group_exists(record) and time.monotonic() < deadline:
            time.sleep(0.2)
    if owned_group_exists(record):
        raise RuntimeError(f"process {record['pid']} did not exit; retained")


def owned_group_exists(record):
    """Find live members of the recorded Linux session and process group.

    Args:
        record: Process metadata including boot and start identity.

    Returns:
        True when a matching group still contains a live process.
    """
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if record.get("boot_id") != boot:
        return False
    leader = identity(record["pid"])
    if leader is not None and leader != record["start"]:
        raise RuntimeError(f"PID {record['pid']} was reused; refusing to signal")
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            stat = path.read_text()
            fields = stat[stat.rfind(")") + 2:].split()
            if (fields[0] != "Z" and int(fields[2]) == record["pid"]
                    and int(fields[3]) == record["pid"]
                    and int(fields[19]) >= int(record["start"])):
                return True
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError, IndexError):
            continue
    return False


def platform(args):
    """Load the selected platform inventory.

    Args:
        args: Parsed command-line options.

    Returns:
        Tuple of platform path and parsed YAML.
    """
    if args.mode == "sim":
        path = ROOT / "src/sim/gazebo/embodied_nexus_sim_gazebo/config/platform_sim.yaml"
    else:
        path = Path(args.config).resolve() if args.config else ROOT / f"src/core/embodied_nexus_core/config/platform_{args.mode}.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("devices"), list):
        raise ValueError(f"invalid platform configuration: {path}")
    return str(path), data


def select_devices(args, data):
    """Resolve IDs and unambiguous type aliases from the inventory.

    Args:
        args: Parsed options.
        data: Platform configuration.

    Returns:
        Selected enabled device dictionaries.
    """
    devices = [item for item in data["devices"] if item.get("enabled", True)]
    if args.all or (args.action == "status" and not args.devices):
        return devices
    selected = []
    for name in args.devices:
        matches = [item for item in devices if item["id"] == name]
        if not matches:
            matches = [item for item in devices if item["type"] == name]
        if len(matches) != 1:
            raise ValueError(f"device {name!r} is unknown or ambiguous; use an instance ID")
        if matches[0] in selected:
            raise ValueError(f"device {name!r} was selected more than once")
        selected.append(matches[0])
    return selected


def api(path, token, body=None, timeout=45):
    """Call the local authenticated Core API.

    Args:
        path: API path.
        token: Bearer token.
        body: Optional JSON request body.
        timeout: Timeout in seconds.

    Returns:
        Decoded JSON response.
    """
    if not token:
        raise ValueError("set EMBODIED_NEXUS_CORE_TOKEN")
    request = Request("http://127.0.0.1:8780" + path,
                      data=None if body is None else json.dumps(body).encode(),
                      headers={"Authorization": "Bearer " + token, "Content-Type": "application/json"})
    try:
        with urlopen(request, timeout=timeout) as response:
            return json.load(response)
    except HTTPError as exc:
        raise RuntimeError(f"Core HTTP {exc.code}: {exc.read().decode(errors='replace')}") from exc
    except URLError as exc:
        raise RuntimeError(f"Core unavailable: {exc.reason}") from exc


def port_busy():
    """Check whether the local Core TCP port is occupied.

    Args:
        None.

    Returns:
        True when port 8780 accepts connections.
    """
    with socket.socket() as sock:
        sock.settimeout(1)
        return sock.connect_ex(("127.0.0.1", 8780)) == 0


def ros(command, timeout=15):
    """Run a ROS CLI command with a bounded timeout.

    Args:
        command: Arguments following ros2.
        timeout: Maximum time in seconds.

    Returns:
        Completed process with captured text.
    """
    return subprocess.run(["ros2", *command], text=True, capture_output=True, timeout=timeout, check=False)


def service_exists(name):
    """Check for an advertised ROS service.

    Args:
        name: Fully qualified service name.

    Returns:
        True if ROS lists the service.
    """
    result = ros(["service", "list"])
    if result.returncode:
        raise RuntimeError(result.stderr or "cannot list ROS services")
    return name in result.stdout.splitlines()


def wait_for(check, process, name, seconds=90):
    """Wait for readiness while checking process ownership.

    Args:
        check: Callable returning whether the service is ready.
        process: Owned process metadata.
        name: Name for diagnostic output.
        seconds: Maximum wait duration.

    Returns:
        None; raises on exit or timeout.
    """
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if not alive(process):
            raise RuntimeError(f"{name} exited; inspect {process['log']}")
        try:
            if check():
                return
        except (RuntimeError, OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(1)
    raise RuntimeError(f"{name} readiness timed out; inspect {process['log']}")


def core_ready(token):
    """Check Core health using the configured token.

    Args:
        token: Core bearer token.

    Returns:
        True if Core reports running.
    """
    return api("/api/v1/core/status", token, timeout=2).get("state") == "running"


def device_service(item):
    """Build the public device service prefix.

    Args:
        item: Device entry from the platform inventory.

    Returns:
        Fully qualified device service prefix.
    """
    return f"/embodied_nexus/{TYPES[item['type']][0]}/{item['id']}"


def readiness_service(item):
    """Return the device's public readiness service.

    Args:
        item: Device inventory entry.

    Returns:
        Fully qualified service name.
    """
    suffix = "/config/get" if item["type"] == "camera" else "/info"
    return device_service(item) + suffix


def config_file(item, platform_path):
    """Resolve the device's public package configuration URI.

    Args:
        item: Device inventory entry.
        platform_path: Path to the platform configuration.

    Returns:
        Absolute device configuration path.
    """
    value = item["config"]
    if value.startswith("package://"):
        package, relative = value[len("package://"):].split("/", 1)
        result = ros(["pkg", "prefix", "--share", package])
        if result.returncode:
            raise RuntimeError(result.stderr)
        return str(Path(result.stdout.strip()) / relative)
    path = Path(value).expanduser()
    return str((path if path.is_absolute() else Path(platform_path).parent / path).resolve())


def validate_device_config(item, platform_path, mode):
    """Reject mismatched device IDs or unsafe mode and backend combinations.

    Args:
        item: Device inventory entry.
        platform_path: Path to the platform configuration.
        mode: Requested mock, real or sim mode.

    Returns:
        Resolved path of the validated device configuration.
    """
    path = config_file(item, platform_path)
    config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(config, dict):
        raise ValueError(f"invalid device configuration: {path}")
    id_key = "camera_id" if item["type"] == "camera" else "device_id"
    if config.get(id_key) != item["id"]:
        raise ValueError(f"device ID mismatch for {item['id']} in {path}")
    backend = config.get("source", {}).get("backend") if item["type"] == "camera" else config.get("backend")
    if not isinstance(backend, str) or not backend:
        raise ValueError(f"missing backend for {item['id']} in {path}")
    if mode == "mock" and backend != "mock":
        raise ValueError(f"{item['id']} uses backend {backend!r}, not mock")
    if mode == "sim" and backend not in {"ros2_sim", "ros2_topics"}:
        raise ValueError(f"{item['id']} uses backend {backend!r}, not simulation")
    if mode == "real" and backend in {"mock", "ros2_sim"}:
        raise ValueError(f"{item['id']} uses backend {backend!r}, not real")
    return path


def stop_standalone(item, record):
    """Stop and verify a standalone device before terminating its launch.

    Args:
        item: Device inventory entry.
        record: Owned launch process metadata.

    Returns:
        None; raises and retains the process if feedback is unconfirmed.
    """
    if not alive(record):
        raise RuntimeError(f"{item['id']} process is not running; stop cannot be confirmed")
    kind = item["type"]
    if kind in {"base", "arm", "gripper"}:
        service_type = f"embodied_nexus_{kind}_interfaces/srv/Stop{TYPES[kind][1]}"
        result = ros(["service", "call", device_service(item) + "/stop", service_type,
                      "{reason: 'service shutdown'}"], timeout=20)
        key = "hardware_stop_confirmed" if kind == "gripper" else "stationary_confirmed"
        if (result.returncode or not re.search(r"\bsuccess=True\b", result.stdout)
                or not re.search(rf"\b{key}=True\b", result.stdout)):
            raise RuntimeError(f"{item['id']} stop not confirmed; process retained: {result.stdout[-400:]} {result.stderr[-200:]}")
    terminate(record)


def run(args, state):
    """Execute a service command against recorded local sessions.

    Args:
        args: Parsed arguments.
        state: Mutable ownership metadata.

    Returns:
        None; raises on unsafe or unsuccessful operations.
    """
    if args.mode == "sim":
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    path, data = platform(args)
    devices = select_devices(args, data)
    mode = "standalone" if args.standalone else "managed"
    key = f"{args.mode}:{args.simulator or '-'}:{mode}"
    session = state.get(key)
    if session and not session.get("processes"):
        del state[key]
        save_state(state)
        session = None
    if session and session.get("config") != path:
        raise RuntimeError("configuration differs from the recorded session")
    if args.action == "status":
        if not session:
            print("No session recorded")
            return
        print(json.dumps({"mode": args.mode, "manager": mode,
                          "processes": {name: {**rec, "running": alive(rec)} for name, rec in session["processes"].items()}}, indent=2))
        if mode == "managed" and port_busy():
            print(json.dumps(api("/api/v1/core/status", os.environ.get("EMBODIED_NEXUS_CORE_TOKEN", "")), indent=2))
        return
    if args.action == "start":
        if mode == "managed" and not os.environ.get("EMBODIED_NEXUS_CORE_TOKEN"):
            raise ValueError("set EMBODIED_NEXUS_CORE_TOKEN")
        configs = {item["id"]: validate_device_config(item, path, args.mode) for item in devices}
        if session is None:
            session = {"config": path, "processes": {}}
        processes = session["processes"]
        env = os.environ.copy()
        token = env.get("EMBODIED_NEXUS_CORE_TOKEN", "")
        if mode == "standalone" and port_busy():
            raise RuntimeError("Core is running; standalone devices cannot be started safely")
        if mode == "managed" and not alive(processes.get("system")):
            if port_busy():
                raise RuntimeError("port 8780 is occupied by an unowned Core or another process")
            if int(data.get("core", {}).get("port", 8780)) != 8780:
                raise ValueError("the selected Core configuration must use port 8780")
            for item in devices:
                if service_exists(readiness_service(item)):
                    raise RuntimeError(f"{item['id']} is already managed by another process")
            if args.mode == "sim":
                if service_exists("/sim/arachne/controller_manager/list_controllers"):
                    raise RuntimeError("a Gazebo controller manager already exists")
                env.setdefault("ROS_LOCALHOST_ONLY", "1")
                if not env.get("IGN_IP"):
                    result = subprocess.run(["hostname", "-I"], capture_output=True, text=True, check=True)
                    env["IGN_IP"] = result.stdout.split()[0]
                command = ["ros2", "launch", "embodied_nexus_sim_gazebo", "sim.launch.py", f"gui:={str(args.gui).lower()}"]
            else:
                command = ["ros2", "run", "embodied_nexus_core", "nexus_core", "--config", path]
            processes["system"] = launch(command, key.replace(":", "-"), env)
            state[key] = session
            save_state(state)
            try:
                if args.mode == "sim":
                    wait_for(lambda: service_exists("/sim/arachne/controller_manager/list_controllers"),
                             processes["system"], "Gazebo")
                wait_for(lambda: core_ready(token), processes["system"], "Core")
            except Exception:
                terminate(processes["system"])
                del state[key]
                save_state(state)
                raise
        if mode == "managed":
            if args.mode == "sim" and not service_exists("/sim/arachne/controller_manager/list_controllers"):
                raise RuntimeError("Gazebo controller manager is not available; Core remains running")
            current = {item["id"]: item for item in api("/api/v1/core/status", token)["devices"]}
            for item in devices:
                existing = current.get(item["id"], {})
                if existing.get("process_state") == "stopped" and service_exists(readiness_service(item)):
                    raise RuntimeError(f"{item['id']} is already managed by another process")
                result = api(f"/api/v1/core/devices/{quote(item['id'], safe='')}", token, {"action": "start"})
                if not result.get("success"):
                    raise RuntimeError(f"failed to start {item['id']}: {result}")
                print(f"started {item['id']}")
        else:
            if args.mode == "sim" and not service_exists("/sim/arachne/controller_manager/list_controllers"):
                raise RuntimeError("Gazebo controller manager is not ready")
            for item in devices:
                if not alive(processes.get(item["id"])) and service_exists(readiness_service(item)):
                    raise RuntimeError(f"{item['id']} is already managed by another process")
            for item in devices:
                name = item["id"]
                if alive(processes.get(name)):
                    print(f"already running: {name}")
                    continue
                ready = readiness_service(item)
                if service_exists(ready):
                    raise RuntimeError(f"{name} is already managed by another process")
                kind = item["type"]
                command = ["ros2", "launch", f"embodied_nexus_{kind}", f"{kind}.launch.py",
                           f"config_file:={configs[name]}", "interface_mode:=ros2",
                           f"use_sim_time:={str(args.mode == 'sim').lower()}"]
                processes[name] = launch(command, f"{args.mode}-{name}", env)
                state[key] = session
                save_state(state)
                wait_for(lambda: service_exists(ready), processes[name], name)
                print(f"started {name}")
        return
    if not session:
        raise RuntimeError("no owned session for the selected mode")
    processes = session["processes"]
    if mode == "managed":
        if not alive(processes.get("system")):
            raise RuntimeError("owned system process is not running; refusing to operate on another Core")
        token = os.environ.get("EMBODIED_NEXUS_CORE_TOKEN", "")
        for item in devices:
            result = api(f"/api/v1/core/devices/{quote(item['id'], safe='')}", token, {"action": "shutdown"})
            if not result.get("success"):
                raise RuntimeError(f"failed to stop {item['id']}: {result}")
            print(f"stopped {item['id']}")
        if args.all:
            result = api("/api/v1/core/shutdown", token, {})
            if not result.get("success"):
                raise RuntimeError(f"Core shutdown refused: {result}")
            terminate(processes["system"])
            del state[key]
    else:
        for item in devices:
            name = item["id"]
            if name not in processes:
                print(f"not owned: {name}")
                continue
            stop_standalone(item, processes[name])
            del processes[name]
            save_state(state)
            print(f"stopped {name}")
        if not processes:
            del state[key]
    save_state(state)


def main():
    """Run the command with an exclusive ownership-state lock.

    Args:
        None.

    Returns:
        Integer process exit status.
    """
    try:
        args = arguments(sys.argv[1:])
        STATE_DIR.mkdir(parents=True, exist_ok=True, mode=0o700)
        with (STATE_DIR / "services.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            state = read_state()
            run(args, state)
        return 0
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.TimeoutExpired, yaml.YAMLError) as exc:
        print(f"services: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
