from setuptools import find_packages, setup
from pathlib import Path

package_name = "embodied_nexus_sim_mujoco"
scene_files = [path.as_posix() for path in Path("config/scenes").glob("*.yaml")]

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/sim.launch.py"]),
        (f"share/{package_name}/config", ["config/platform_sim.yaml"]),
        (f"share/{package_name}/config/scenes", scene_files),
    ],
    install_requires=["setuptools", "mujoco==3.2.7", "numpy", "PyYAML"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="MuJoCo simulation bridge for EmbodiedNexus",
    license="MIT",
    entry_points={
        "console_scripts": ["mujoco_sim = embodied_nexus_sim_mujoco.simulator:main"],
        "embodied_nexus_core.scene_editors": [
            "mujoco = embodied_nexus_sim_mujoco.scene:SceneLibrary",
        ],
    },
)
