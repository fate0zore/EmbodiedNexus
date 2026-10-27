from embodied_nexus_client import LidarClient


def main() -> None:
    """解析启动参数并运行程序入口。

    Args:
        无参数。

    Returns:
        无返回值。
    """
    client = LidarClient("http://robot-host:8777", token="replace-with-a-local-token")
    print(client.info())
    print(client.status())
    with client.acquire_control("lidar-demo") as lease:
        print(lease.start_motor())
        print(lease.set_speed(10))
        for frame in client.subscribe(streams=("points", "scan"), point_stride=4):
            print(frame.metadata["channel"], frame.metadata.get("frame_id"), len(frame.data))


if __name__ == "__main__":
    main()
