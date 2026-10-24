from setuptools import find_packages, setup

package_name = "embodied_nexus_lidar"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/c16.yaml", "config/c16_topics.yaml", "config/c16_mock.yaml", "config/c16_sim.yaml"]),
        ("share/" + package_name + "/launch", ["launch/lidar.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML", "starlette", "uvicorn"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Configurable C16 ROS 2 lidar adapter and authenticated Web gateway",
    license="MIT",
    entry_points={"console_scripts": [
        "lidar_node = embodied_nexus_lidar.lidar_node:main",
        "lidar_web = embodied_nexus_lidar.ros_gateway:main",
        "lidar_check = embodied_nexus_lidar.cli:main",
    ]},
)
