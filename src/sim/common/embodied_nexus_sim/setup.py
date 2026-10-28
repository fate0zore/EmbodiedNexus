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
        "embodied_nexus_base.backends": ["scout_2.ros2_sim = embodied_nexus_sim.adapters.scout_2.backend:Ros2SimBaseBackend"],
        "embodied_nexus_arm.backends": ["aubo_i5.ros2_sim = embodied_nexus_sim.adapters.aubo_i5.backend:Ros2SimArmBackend"],
        "embodied_nexus_gripper.backends": ["ms42dc.ros2_sim = embodied_nexus_sim.adapters.ms42dc.backend:Ros2SimGripperBackend"],
    },
)
