from setuptools import find_packages, setup

package_name = "embodied_nexus_arm"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/aubo_i5_mock.yaml", "config/aubo_i5_real.yaml", "config/aubo_i5_sim.yaml"]),
        ("share/" + package_name + "/launch", ["launch/arm.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML", "starlette", "uvicorn"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Aubo i5 arm adapter with ROS 2 and authenticated Web gateway",
    license="MIT",
    entry_points={"console_scripts": [
        "arm_node = embodied_nexus_arm.arm_node:main",
        "arm_web = embodied_nexus_arm.ros_gateway:main",
        "arm_check = embodied_nexus_arm.cli:main",
    ]},
)
