from setuptools import find_packages, setup

package_name = "embodied_nexus_gripper"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/ms42dc.yaml", "config/ms42dc_mock.yaml", "config/ms42dc_sim.yaml"]),
        ("share/" + package_name + "/launch", ["launch/gripper.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML", "starlette", "uvicorn"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="ROS 2 MS42DC gripper adapter and authenticated HTTP gateway",
    license="MIT",
    entry_points={"console_scripts": [
        "gripper_node = embodied_nexus_gripper.gripper_node:main",
        "gripper_web = embodied_nexus_gripper.gripper_web_node:main",
        "gripper_check = embodied_nexus_gripper.cli:main",
    ]},
)
