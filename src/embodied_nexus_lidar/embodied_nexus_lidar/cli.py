from __future__ import annotations

import argparse
import difflib
import json
import os
from pathlib import Path
import socket

from .config import config_json, load_config


def check_driver_available(config: dict) -> None:
    if os.name != "posix":
        raise RuntimeError("the C16 UDP driver requires the Ubuntu/Linux ROS 2 runtime")
    current_pid = os.getpid()
    proc = Path("/proc")
    if proc.is_dir():
        for entry in proc.iterdir():
            if not entry.name.isdigit() or int(entry.name) in {current_pid, os.getppid()}:
                continue
            try:
                argv = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="ignore")
            except (OSError, PermissionError):
                continue
            if "lslidar_driver_node" in argv:
                raise RuntimeError(f"existing lslidar_driver process {entry.name} may own the configured UDP ports")
    driver = config["driver"]
    opened = []
    try:
        for label in ("msop_port", "difop_port"):
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("0.0.0.0", int(driver[label])))
            except OSError as exc:
                sock.close()
                raise RuntimeError(f"UDP port {driver[label]} ({label}) is already in use") from exc
            opened.append(sock)
    finally:
        for sock in opened:
            sock.close()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Validate and inspect C16 lidar configuration")
    parser.add_argument("--config", default=None)
    parser.add_argument("--show-config", action="store_true")
    parser.add_argument("--diff-from", default=None)
    parser.add_argument("--check-hardware", action="store_true", help="read-only process and UDP-port checks")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.check_hardware and config["backend"] == "c16":
            check_driver_available(config)
        if args.diff_from:
            before = config_json(load_config(args.diff_from)).splitlines(keepends=True)
            after = config_json(config).splitlines(keepends=True)
            print("".join(difflib.unified_diff(before, after, fromfile=args.diff_from,
                                              tofile=args.config or "defaults")))
        if args.show_config:
            print(config_json(config))
        if not args.show_config and not args.diff_from:
            print(json.dumps({"ok": True, "device_id": config["device_id"], "model": config["model"],
                              "backend": config["backend"], "interface_mode": config["interface_mode"],
                              "capture_starts": config["backend"] != "ros2_topics",
                              "motor_command_sent_at_start": False}, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        parser.error(str(exc))
        return 2
