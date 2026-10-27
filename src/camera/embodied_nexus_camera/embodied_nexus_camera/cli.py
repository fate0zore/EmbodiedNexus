from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_config, validate_config


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description="Validate EmbodiedNexus camera configuration")
    parser.add_argument("--config", default=None)
    parser.add_argument("--dry-run-check", action="store_true")
    parser.add_argument("--check-hardware", action="store_true")
    args = parser.parse_args(argv)
    config = load_config(args.config)
    validate_config(config)
    if args.check_hardware and config["source"]["backend"] == "gemini_v4l2":
        for name in ("color_device", "depth_device"):
            device = Path(config["source"][name]).expanduser()
            if not device.exists():
                parser.error(f"configured {name} does not exist: {device}")
    print(json.dumps({"ok": True, "camera_id": config["camera_id"], "backend": config["source"]["backend"], "interface_mode": config["interface_mode"]}, indent=2))
    return 0
