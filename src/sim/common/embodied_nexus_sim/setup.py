from setuptools import setup

setup(
    name="embodied_nexus_sim",
    version="0.1.0",
    packages=["embodied_nexus_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/embodied_nexus_sim"]),
        ("share/embodied_nexus_sim", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "embodied_nexus_base.backends": ["ros2_sim = embodied_nexus_sim.base:Ros2SimBaseBackend"],
        "embodied_nexus_arm.backends": ["ros2_sim = embodied_nexus_sim.arm:Ros2SimArmBackend"],
        "embodied_nexus_gripper.backends": ["ros2_sim = embodied_nexus_sim.gripper:Ros2SimGripperBackend"],
    },
)
