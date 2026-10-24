from setuptools import find_packages, setup

package_name = "embodied_nexus_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gemini335.yaml", "config/gemini335_mock.yaml", "config/gemini335_sim.yaml", "config/gemini335_factory_calibration.json"]),
        ("share/" + package_name + "/launch", ["launch/camera.launch.py"]),
        ("share/" + package_name + "/test", ["test/test_camera_core.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Configurable ROS 2 RGB-D camera adapter and Web gateway",
    license="MIT",
    entry_points={
        "console_scripts": [
            "camera_node = embodied_nexus_camera.camera_node:main",
            "camera_web = embodied_nexus_camera.web_server:main",
            "camera_check = embodied_nexus_camera.cli:main",
        ],
    },
)
