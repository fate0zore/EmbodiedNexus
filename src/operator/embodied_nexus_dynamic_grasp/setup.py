from glob import glob

from setuptools import find_packages, setup

package_name = "embodied_nexus_dynamic_grasp"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "torch>=2.5", "ultralytics>=8.3.253"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Independent dynamic target grasping components",
    license="MIT",
    entry_points={"console_scripts": [
        "dynamic_grasp_perception = embodied_nexus_dynamic_grasp.perception_node:main",
        "dynamic_grasp_server = embodied_nexus_dynamic_grasp.grasp_node:main",
        "dynamic_grasp_acceptance = embodied_nexus_dynamic_grasp.acceptance:main",
        "dynamic_grasp_fixture = embodied_nexus_dynamic_grasp.motion_fixture:main",
        "dynamic_grasp_demo = embodied_nexus_dynamic_grasp.demo:main",
    ]},
)
