"""Translate the shared, expanded Arachne URDF to a MuJoCo scene."""

from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path


def _pose(element):
    """Convert a URDF origin to MJCF position and Euler attributes.

    Args:
        element: Optional URDF origin element.

    Returns:
        Dictionary of MJCF pose attributes.
    """
    if element is None:
        return {}
    return {"pos": element.get("xyz", "0 0 0"), "euler": element.get("rpy", "0 0 0")}


def make_scene(urdf, description_dir, camera_calibration=None):
    """Convert expanded Arachne URDF collision geometry and dynamics to MJCF.

    Args:
        urdf: Expanded URDF XML text.
        description_dir: Installed shared description package directory.
        camera_calibration: Optional shared Gemini 335 calibration data.

    Returns:
        MJCF XML text with a free mobile base and workcell geometry.
    """
    robot = ET.fromstring(urdf)
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").attrib["link"], []).append(joint)
    scene = ET.Element("mujoco", {"model": "arachne"})
    # URDF RPY rotates about fixed X, Y, Z axes; MuJoCo's uppercase sequence
    # uses those same fixed axes for every imported joint and geometry origin.
    ET.SubElement(scene, "compiler", {"angle": "radian", "eulerseq": "XYZ",
                                     "autolimits": "true", "balanceinertia": "true"})
    ET.SubElement(scene, "option", {"timestep": "0.002", "gravity": "0 0 -9.8",
                                     "integrator": "implicitfast"})
    asset = ET.SubElement(scene, "asset")
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 0 8", "dir": "-0.4 0.2 -0.9"})
    ET.SubElement(world, "geom", {"name": "ground", "type": "plane", "size": "20 20 0.1", "rgba": "0.75 0.75 0.75 1"})
    ET.SubElement(world, "geom", {"name": "obstacle", "type": "box", "pos": "2.6 1.2 1", "size": "0.175 0.4 1", "rgba": "0.8 0.15 0.08 1"})
    actuator = ET.SubElement(scene, "actuator")
    mesh_index = 0

    def add_link(parent, link_name, joint=None):
        """Append one link and its descendants to the MJCF body tree.

        Args:
            parent: Parent MJCF body or worldbody.
            link_name: Name of the source URDF link.
            joint: Optional joint connecting this link to its parent.

        Returns:
            None.
        """
        nonlocal mesh_index
        attrs = {"name": link_name}
        if link_name.startswith(("aubo_", "ms42dc_")):
            attrs["gravcomp"] = "1"
        if joint is None:
            attrs["pos"] = "0 0 0.236"
        else:
            attrs.update(_pose(joint.find("origin")))
        body = ET.SubElement(parent, "body", attrs)
        if joint is None:
            ET.SubElement(body, "freejoint", {"name": "root"})
        elif joint.get("type") != "fixed":
            kind = joint.attrib["type"]
            if kind not in {"continuous", "revolute", "prismatic"}:
                raise ValueError(f"unsupported joint type: {kind}")
            axis = joint.find("axis")
            ja = {"name": joint.attrib["name"], "type": "slide" if kind == "prismatic" else "hinge",
                  "axis": axis.get("xyz", "1 0 0") if axis is not None else "1 0 0",
                  "damping": "4", "armature": "0.1"}
            limit = joint.find("limit")
            if kind != "continuous" and limit is not None:
                ja["range"] = f"{limit.attrib['lower']} {limit.attrib['upper']}"
            ET.SubElement(body, "joint", ja)
            if "wheel" in ja["name"]:
                ET.SubElement(actuator, "velocity", {"name": ja["name"], "joint": ja["name"], "kv": "20", "forcelimited": "true", "forcerange": "-100 100"})
            else:
                position_actuator = {"name": ja["name"], "joint": ja["name"],
                                     "kp": "50", "kv": "10", "forcelimited": "true",
                                     "forcerange": "-300 300"}
                if "range" in ja:
                    position_actuator.update({"ctrllimited": "true", "ctrlrange": ja["range"]})
                ET.SubElement(actuator, "position", position_actuator)
        link = links[link_name]
        inertial = link.find("inertial")
        # Scout places the chassis inertia on a fixed, coincident child link.
        if link_name == "base_link" and inertial is None and "inertial_link" in links:
            inertial = links["inertial_link"].find("inertial")
        if inertial is not None:
            inertia = inertial.find("inertia")
            values = [inertia.get(k, "0") for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
            ia = {"mass": inertial.find("mass").attrib["value"], "fullinertia": " ".join(values)}
            ia.update(_pose(inertial.find("origin")))
            # MuJoCo fullinertia is expressed in the body frame and does not accept an orientation.
            ia.pop("euler", None)
            ET.SubElement(body, "inertial", ia)
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            ga = {"rgba": "0.55 0.58 0.62 1", "mass": "0", "group": "1",
                  "contype": "0", "conaffinity": "0"}
            ga.update(_pose(collision.find("origin")))
            shape = geometry[0]
            if shape.tag == "mesh":
                name = f"mesh_{mesh_index}"
                mesh_index += 1
                uri = shape.attrib["filename"]
                prefix = "package://embodied_nexus_sim_description/"
                if not uri.startswith(prefix):
                    raise ValueError(f"unexpected mesh URI: {uri}")
                path = (Path(description_dir) / uri[len(prefix):]).resolve()
                ma = {"name": name, "file": str(path)}
                if "scale" in shape.attrib:
                    ma["scale"] = shape.attrib["scale"]
                ET.SubElement(asset, "mesh", ma)
                ga.update({"type": "mesh", "mesh": name})
            elif shape.tag == "box":
                ga.update({"type": "box", "size": " ".join(str(float(v) / 2) for v in shape.attrib["size"].split())})
            elif shape.tag == "cylinder":
                ga.update({"type": "cylinder", "size": f"{shape.attrib['radius']} {float(shape.attrib['length']) / 2}"})
            elif shape.tag == "sphere":
                ga.update({"type": "sphere", "size": shape.attrib["radius"]})
            else:
                raise ValueError(f"unsupported geometry: {shape.tag}")
            ET.SubElement(body, "geom", ga)
        if camera_calibration is not None and link_name in {"camera_color_frame", "camera_depth_frame"}:
            channel = "color" if link_name == "camera_color_frame" else "depth"
            stream = camera_calibration["streams"][channel]
            width, height = stream["width"], stream["height"]
            # MuJoCo camera local +X is right, +Y is up, and -Z is forward.
            # The URDF camera frame is +X forward, +Y left, and +Z up.
            # MuJoCo principalpixel is a displacement of the frustum from
            # image center, with Y pointing up; CameraInfo uses top-left pixels.
            ET.SubElement(body, "camera", {
                "name": f"camera_{channel}",
                "xyaxes": "0 -1 0 0 0 1",
                "resolution": f"{width} {height}",
                "sensorsize": f"{width} {height}",
                "focalpixel": f"{stream['fx']} {stream['fy']}",
                "principalpixel": f"{width / 2 - stream['cx']} {stream['cy'] - height / 2}",
            })
        for child in children.get(link_name, []):
            if link_name == "base_link" and child.find("child").get("link") == "inertial_link":
                continue
            add_link(body, child.find("child").attrib["link"], child)

    add_link(world, "base_link")
    return ET.tostring(scene, encoding="unicode")
