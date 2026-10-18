from __future__ import annotations

import argparse
import difflib
import json

from .config import config_json, load_config


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Inspect and validate Scout base configuration")
    parser.add_argument("--config", default=None)
    parser.add_argument("--show-config", action="store_true")
    parser.add_argument("--diff-from", default=None)
    args = parser.parse_args(argv)
    try:
        config = load_config(args.config)
        if args.diff_from:
            before = config_json(load_config(args.diff_from)).splitlines(keepends=True)
            after = config_json(config).splitlines(keepends=True)
            print("".join(difflib.unified_diff(before, after, fromfile=args.diff_from,
                                              tofile=args.config or "defaults")))
        if args.show_config:
            print(config_json(config))
        if not args.show_config and not args.diff_from:
            print(json.dumps({"ok": True, "device_id": config["device_id"],
                              "model": config["model"], "backend": config["backend"],
                              "interface_mode": config["interface_mode"],
                              "motion_enabled_at_start": False}, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        parser.error(str(exc))
        return 2
