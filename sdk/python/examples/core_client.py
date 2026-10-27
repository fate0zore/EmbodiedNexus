from __future__ import annotations

import argparse
import json
import os

from embodied_nexus_client import CoreClient


def main() -> int:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        本次处理产生的结果值。 返回类型为 int。
    """
    parser = argparse.ArgumentParser(description="Inspect and manage EmbodiedNexus devices through core")
    parser.add_argument("action", nargs="?", choices=("status", "start", "shutdown", "stop-all", "camera-info"),
                        default="status")
    parser.add_argument("--device-id", help="configured platform device ID")
    args = parser.parse_args()

    core = CoreClient(
        os.environ["EMBODIED_NEXUS_CORE_URL"],
        os.environ["EMBODIED_NEXUS_CORE_TOKEN"],
    )
    if args.action == "status":
        result = core.status()
    elif args.action in {"start", "shutdown", "camera-info"}:
        if not args.device_id:
            parser.error(f"{args.action} requires --device-id")
        if args.action == "start":
            result = core.start(args.device_id)
        elif args.action == "shutdown":
            result = core.shutdown(args.device_id)
        else:
            result = core.camera(args.device_id).info()
    else:
        result = core.stop_all()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
