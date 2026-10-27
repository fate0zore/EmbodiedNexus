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
    """解析启动参数并运行程序入口。

    Args:
        argv: argv，作为解析启动参数并运行程序入口所需的输入。

    Returns:
        无返回值。
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=None)
    args, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = GripperRosGateway(config_path=args.config)
    web_config = node._config_data["web"]
    token_env = str(web_config["auth_token_env"])
    token = os.environ.get(token_env, "")
    app = create_app(node, token)
    server = uvicorn.Server(uvicorn.Config(
        app, host=str(web_config["bind_host"]), port=int(web_config["port"]),
        log_level="info", access_log=True,
    ))
    thread = threading.Thread(target=server.run, name="gripper-http", daemon=True)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread.start()
    node.get_logger().info(
        f"MS42DC HTTP API listening on {web_config['bind_host']}:{web_config['port']}"
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
