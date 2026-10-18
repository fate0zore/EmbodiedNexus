from __future__ import annotations

import argparse
import os
import threading

import rclpy
import uvicorn
from rclpy.executors import MultiThreadedExecutor

from .ros_gateway import GripperRosGateway
from .web_server import create_app


def main(argv=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = GripperRosGateway(config_path=args.config)
    token_env = str(node.config["web"]["auth_token_env"])
    token = os.environ.get(token_env, "")
    app = create_app(node, token)
    server = uvicorn.Server(uvicorn.Config(
        app, host=str(node.config["web"]["bind_host"]), port=int(node.config["web"]["port"]),
        log_level="info", access_log=True,
    ))
    thread = threading.Thread(target=server.run, name="gripper-http", daemon=True)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread.start()
    node.get_logger().info(
        f"MS42DC HTTP API listening on {node.config['web']['bind_host']}:{node.config['web']['port']}"
    )
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        if thread.is_alive():
            thread.join(timeout=3.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
