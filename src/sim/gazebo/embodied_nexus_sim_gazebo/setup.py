from setuptools import find_packages, setup

package_name = "embodied_nexus_sim_gazebo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/sim.launch.py"]),
        ("share/" + package_name + "/config", ["config/controllers.yaml", "config/platform_sim.yaml"]),
        ("share/" + package_name + "/worlds", ["worlds/workcell.sdf"]),
        ("share/" + package_name + "/docs", ["docs/gazebo-fortress.md"]),
        ("share/" + package_name + "/examples", [
            "examples/validate_sim.py", "examples/validate_clock_rewind.py",
        ]),
        ("share/" + package_name + "/test", [
            "test/test_sim_assets.py", "test/test_sensor_relay.py", "test/test_clock_guard.py",
            "test/test_performance_profiles.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Gazebo Fortress integration for EmbodiedNexus device backends",
    license="MIT",
    entry_points={"console_scripts": [
        "sim_camera_info = embodied_nexus_sim_gazebo.camera_info:main",
        "sim_sensor_relay = embodied_nexus_sim_gazebo.sensor_relay:main",
        "sim_clock_guard = embodied_nexus_sim_gazebo.clock_guard:main",
    ]},
)
