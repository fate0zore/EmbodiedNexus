#!/usr/bin/env python3
"""Read Aubo state and configuration once; this script sends no control commands."""

from __future__ import annotations

import argparse
import json

from embodied_nexus_arm.adapters.aubo_i5.backend import AuboBackend
from embodied_nexus_arm.config import load_config


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=None,
        help="real Aubo YAML (defaults to the installed aubo_i5_real.yaml)",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if config["backend"] != "aubo_jsonrpc":
        parser.error("the read-only probe requires backend: aubo_jsonrpc")

    backend = AuboBackend(config)
    try:
        backend.open()
        feedback = backend.poll()
        report = {
            "device_id": config["device_id"],
            "model": config["model"],
            "connected": backend.connected,
            "positions_rad": feedback["positions_rad"],
            "velocities_rad_sec": feedback["velocities_rad_sec"],
            "mode": feedback["mode"],
            "safety_mode": feedback["safety_mode"],
            "teach_enabled": feedback["teach_enabled"],
            "hardware_estop": feedback["hardware_estop"],
            "joint_limits_rad": {
                "min": backend.hardware_min,
                "max": backend.hardware_max,
            },
            "payload": backend.payload,
            "tcp_offset": backend.tcp_offset,
            "last_error": backend.last_error,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    finally:
        backend.close()


if __name__ == "__main__":
    raise SystemExit(main())
