from __future__ import annotations

import argparse
import json

from .config import config_json, load_config


def main(argv=None) -> int:
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description="Inspect and validate Aubo i5 arm configuration")
    parser.add_argument("--config", default=None)
    parser.add_argument("--show-config", action="store_true")
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.show_config:
            print(config_json(config))
        else:
            print(json.dumps({"ok": True, "device_id": config["device_id"],
                              "model": config["model"], "backend": config["backend"],
                              "interface_mode": config["interface_mode"],
                              "motion_enabled_at_start": False}, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        parser.error(str(exc))
        return 2
