from __future__ import annotations

import argparse
import os

from embodied_nexus_camera_client import GripperClient


def main() -> None:
    parser = argparse.ArgumentParser(description="Run a bounded MS42DC open/close/relative command")
    parser.add_argument("--url", required=True, help="robot base URL, e.g. http://robot-host:8776")
    parser.add_argument("--device-id", default="gripper_main")
    parser.add_argument("command", choices=("open", "close", "relative"))
    parser.add_argument("--angle-degrees", type=float, default=10.0,
                        help="relative motor angle; positive closes and negative opens")
    args = parser.parse_args()
    token = os.environ.get("EMBODIED_NEXUS_GRIPPER_TOKEN", "")
    if not token:
        parser.error("set EMBODIED_NEXUS_GRIPPER_TOKEN before running this example")
    client = GripperClient(args.url, token, args.device_id)
    with client.acquire_control("gripper-control-example") as lease:
        lease.enable_motion()
        if args.command == "open":
            operation = lease.move_open()
        elif args.command == "close":
            operation = lease.move_close()
        else:
            operation = lease.move_relative(args.angle_degrees)
        result = operation.wait(timeout=12.0)
        print(result)
        if not result.get("success"):
            raise SystemExit(1)


if __name__ == "__main__":
    main()
