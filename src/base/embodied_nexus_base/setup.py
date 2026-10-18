from setuptools import find_packages, setup

package_name = "embodied_nexus_base"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/scout_mock.yaml", "config/scout_waveshare.yaml"]),
        ("share/" + package_name + "/launch", ["launch/base.launch.py"]),
    ],
    install_requires=["setuptools", "PyYAML", "pyserial", "starlette", "uvicorn"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Configurable Scout 2.0 base adapter and authenticated Web gateway",
    license="MIT",
    entry_points={"console_scripts": [
        "base_node = embodied_nexus_base.base_node:main",
        "base_web = embodied_nexus_base.ros_gateway:main",
        "base_check = embodied_nexus_base.cli:main",
    ]},
)
