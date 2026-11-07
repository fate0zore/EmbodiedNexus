from setuptools import find_packages, setup

package_name = "embodied_nexus_operator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "requirements-vision.txt"]),
        ("share/" + package_name + "/launch", ["launch/operator.launch.py", "launch/grasp_demo.launch.py"]),
        ("share/" + package_name + "/config", ["config/grasp.yaml", "config/grasp_cylinder.yaml"]),
        ("share/" + package_name + "/scripts", ["scripts/prepare_yoloe.py"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Planned visual grasp algorithm for MuJoCo and Nexus device services",
    license="MIT",
    entry_points={"console_scripts": [
        "grasp_operator = embodied_nexus_operator.grasp_node:main",
        "grasp_acceptance = embodied_nexus_operator.acceptance:main",
        "preview_yoloe = embodied_nexus_operator.preview_yoloe:main",
    ]},
)
