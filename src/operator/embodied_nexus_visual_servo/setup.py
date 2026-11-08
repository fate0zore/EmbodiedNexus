from setuptools import find_packages, setup

package_name = "embodied_nexus_visual_servo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/visual_servo.yaml", "config/servo.yaml",
                                                "config/arm_sim_servo.yaml", "config/gripper_sim_servo.yaml"]),
        ("share/" + package_name + "/launch", ["launch/visual_servo.launch.py"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "torch>=2.5", "ultralytics>=8.3.253"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="Independent visual servo grasping components",
    license="MIT",
    entry_points={"console_scripts": [
        "visual_servo_perception = embodied_nexus_visual_servo.perception_node:main",
        "visual_servo_grasp = embodied_nexus_visual_servo.grasp_node:main",
        "visual_servo_acceptance = embodied_nexus_visual_servo.acceptance:main",
    ]},
)
