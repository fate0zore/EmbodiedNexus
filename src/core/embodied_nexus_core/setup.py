from setuptools import find_packages, setup

package_name = "embodied_nexus_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/platform_mock.yaml", "config/platform_real.yaml",
        ]),
    ],
    install_requires=["setuptools", "PyYAML", "starlette>=1.3.1", "uvicorn>=0.15", "websockets>=10.4"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Unified EmbodiedNexus device process manager and authenticated gateway",
    license="MIT",
    entry_points={"console_scripts": ["nexus_core = embodied_nexus_core.core_node:main"]},
)
