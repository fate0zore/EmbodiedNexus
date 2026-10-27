from __future__ import annotations

import argparse
import difflib
import json
from pathlib import Path

from .config import config_json, load_config


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description="Validate and inspect MS42DC gripper configuration")
    parser.add_argument("--config", default=None)
    parser.add_argument("--show-config", action="store_true", help="print the final merged configuration")
    parser.add_argument("--diff-from", default=None, help="compare the final configuration with another YAML file")
    parser.add_argument("--check-hardware", action="store_true", help="check that the configured serial port exists")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.check_hardware and config["backend"] == "serial":
            port = Path(config["serial"]["port"]).expanduser()
            if not port.exists():
                raise ValueError(f"configured serial port does not exist: {port}")
        if args.diff_from:
            previous = config_json(load_config(args.diff_from)).splitlines(keepends=True)
            current = config_json(config).splitlines(keepends=True)
            print("".join(difflib.unified_diff(previous, current, fromfile=args.diff_from, tofile=args.config or "defaults")))
        if args.show_config:
            print(config_json(config))
        if not args.show_config and not args.diff_from:
            print(json.dumps({
                "ok": True, "device_id": config["device_id"], "model": config["model"],
                "backend": config["backend"], "interface_mode": config["interface_mode"],
                "motion_enabled_at_start": False,
                "feedback_confirmed": config["feedback"]["confirmed"],
            }, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        parser.error(str(exc))
        return 2
