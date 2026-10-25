from pathlib import Path
from setuptools import find_packages, setup

package_name = "embodied_nexus_core"
rviz_dist = Path("web_rviz/dist")
if not (rviz_dist / "index.html").is_file():
    raise RuntimeError("Web RViz assets are missing; run npm ci and npm run build in src/core/embodied_nexus_core/web_rviz")
rviz_data_files = []
for asset in rviz_dist.rglob("*"):
    if asset.is_file():
        destination = Path("share") / package_name / "web" / "rviz" / asset.relative_to(rviz_dist).parent
        rviz_data_files.append((destination.as_posix(), [asset.as_posix()]))

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
        ("share/" + package_name + "/web", [
            "web/index.html", "web/app.css", "web/app.js", "web/commands.json",
        ]),
    ] + rviz_data_files,
    install_requires=["setuptools", "PyYAML", "starlette>=1.3.1", "uvicorn>=0.15", "websockets>=10.4"],
    zip_safe=True,
    maintainer="EmbodiedNexus",
    maintainer_email="maintainer@example.com",
    description="EmbodiedNexus device manager, authenticated API gateway and Web console",
    license="MIT",
    entry_points={"console_scripts": ["nexus_core = embodied_nexus_core.core_node:main"]},
)
