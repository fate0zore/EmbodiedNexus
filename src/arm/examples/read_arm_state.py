#!/usr/bin/env python3
"""Read the latest Aubo state from an already running EmbodiedNexus service."""

import argparse
import json
import os

from embodied_nexus_client import ArmClient


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--url", default="http://jetson.local:8779")
    parser.add_argument("--device-id", default="arm_main")
    args = parser.parse_args()
    token = os.environ.get("EMBODIED_NEXUS_ARM_TOKEN", "")
    if not token:
        parser.error("EMBODIED_NEXUS_ARM_TOKEN is required")
    state = ArmClient(args.url, token, device_id=args.device_id).status()
    print(json.dumps(state, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
