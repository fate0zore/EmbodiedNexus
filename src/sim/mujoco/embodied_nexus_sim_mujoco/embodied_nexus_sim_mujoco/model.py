"""Translate the shared, expanded Arachne URDF to a MuJoCo scene."""

from __future__ import annotations

import xml.etree.ElementTree as ET
import math
import json
from .fin_ray import add_fin_ray
from pathlib import Path
import struct


DEFAULT_SCENE = {
    "name": "Default",
    "ground": {"size": [20.0, 20.0, 0.1], "rgba": [0.75, 0.75, 0.75, 1.0]},
    "objects": [{
        "id": "obstacle", "shape": "box", "position": [2.6, 1.2, 1.0],
        "rotation": [0.0, 0.0, 0.0], "size": [0.35, 0.8, 2.0],
        "rgba": [0.8, 0.15, 0.08, 1.0], "motion": "static",
    }],
}
ARM_JOINT_NAMES = {
    "aubo_shoulder_joint", "aubo_upperArm_joint", "aubo_foreArm_joint",
    "aubo_wrist1_joint", "aubo_wrist2_joint", "aubo_wrist3_joint",
}


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


def _scaled_mesh_bounds_center(path, scale):
    """Return the axis-aligned mesh-bounds center after applying mesh scale.

    Args:
        path: Resolved path to a binary or ASCII STL mesh.
        scale: XYZ scale applied to mesh vertices.

    Returns:
        Scaled XYZ center of the mesh bounds in meters.
    """
    data = Path(path).read_bytes()
    vertices = []
    triangle_count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else -1
    if triangle_count >= 0 and len(data) == 84 + triangle_count * 50:
        for triangle_index in range(triangle_count):
            triangle_offset = 84 + triangle_index * 50 + 12
            for vertex_index in range(3):
                vertices.append(struct.unpack_from("<3f", data, triangle_offset + vertex_index * 12))
    else:
        for line in data.decode("utf-8", errors="ignore").splitlines():
            fields = line.split()
            if len(fields) == 4 and fields[0].lower() == "vertex":
                vertices.append(tuple(float(value) for value in fields[1:]))
    if not vertices:
        raise ValueError(f"finger STL mesh contains no vertices: {path}")
    return [
        (min(float(point[index]) for point in vertices) +
         max(float(point[index]) for point in vertices)) * 0.5 * float(scale[index])
        for index in range(3)
    ]


def make_scene(urdf, description_dir, camera_calibration=None, scene_config=None,
               scene_directory=None, simplified_finger_collision=True,
               finger_contact_box_half_size_m=(0.023, 0.052, 0.012)):
    """Convert expanded Arachne URDF collision geometry and dynamics to MJCF.

    Args:
        urdf: Expanded URDF XML text.
        description_dir: Installed shared description package directory.
        camera_calibration: Optional shared Gemini 335 calibration data.
        scene_config: Optional validated workspace scene mapping; it may define
            finger collision geometry, contact material, and contact friction cone.
        scene_directory: Directory containing the scene YAML and relative mesh assets.
        simplified_finger_collision: Whether to use adjustable boxes for finger contact.
        finger_contact_box_half_size_m: Three positive half dimensions for each contact box.

    Returns:
        MJCF XML text with the selected workcell geometry and free mobile base.
    """
    robot = ET.fromstring(urdf)
    selected_scene = scene_config or DEFAULT_SCENE
    finger_geometry = selected_scene.get("finger_collision_geometry")
    if finger_geometry not in (None, "mesh", "boxes", "convex"):
        raise ValueError("finger_collision_geometry must be mesh, boxes or convex")
    if finger_geometry is not None:
        simplified_finger_collision = finger_geometry == "boxes"
    finger_material = selected_scene.get("finger_contact_material", {})
    flex_pads = selected_scene.get("finger_flex_pads", {})
    if flex_pads.get("enabled") and not simplified_finger_collision:
        raise ValueError("flexible finger pads require simplified finger collision boxes")
    # Higher geom priority makes the finger's contact properties govern its
    # contact with scene objects, instead of averaging solver parameters.
    finger_material_attrs = {"priority": "1"} if finger_material else {}
    for key in ("friction", "solref", "solimp"):
        if key in finger_material:
            finger_material_attrs[key] = " ".join(str(float(value)) for value in finger_material[key])
    if "condim" in finger_material:
        finger_material_attrs["condim"] = str(finger_material["condim"])
    contact_half_size = [float(value) for value in selected_scene.get(
        "finger_contact_box_half_size_m", finger_contact_box_half_size_m)]
    if (len(contact_half_size) != 3 or
            any(not math.isfinite(value) or value <= 0.0 for value in contact_half_size)):
        raise ValueError("finger contact box half sizes must be three positive finite values")
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").attrib["link"], []).append(joint)
    scene = ET.Element("mujoco", {"model": "arachne"})
    # URDF RPY rotates about fixed X, Y, Z axes; MuJoCo's uppercase sequence
    # uses those same fixed axes for every imported joint and geometry origin.
    ET.SubElement(scene, "compiler", {"angle": "radian", "eulerseq": "XYZ",
                                     "autolimits": "true", "balanceinertia": "true"})
    contact_cone = selected_scene.get("contact_cone", "pyramidal")
    if contact_cone not in ("pyramidal", "elliptic"):
        raise ValueError("contact_cone must be pyramidal or elliptic")
    timestep = 0.0005 if flex_pads.get("enabled") else 0.002
    if flex_pads.get("enabled") and flex_pads.get("style") == "fin_ray":
        # MuJoCo clamps positive solref time constants to at least twice the step.
        timestep = min(timestep, float(flex_pads.get("fin_ray_edge_time_constant_s", 0.0005)) / 2.0)
    option = {"timestep": str(timestep), "gravity": "0 0 -9.8",
              "integrator": "implicitfast", "cone": contact_cone}
    option["noslip_iterations"] = str(selected_scene.get("noslip_iterations", 0))
    if flex_pads.get("enabled") and flex_pads.get("style") == "fin_ray":
        option.update({"solver": "CG", "iterations": "50"})
    ET.SubElement(scene, "option", option)
    if selected_scene.get("fixed_base", False):
        equality = ET.SubElement(scene, "equality")
        ET.SubElement(equality, "weld", {"body1": "base_link", "solref": "0.004 1"})
    asset = ET.SubElement(scene, "asset")
    world = ET.SubElement(scene, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 0 8", "dir": "-0.4 0.2 -0.9"})
    finger_contact_center_offset = [
        float(value) for value in selected_scene.get("finger_contact_center_offset_m", [0.0, 0.0, 0.0])
    ]
    if (len(finger_contact_center_offset) != 3 or
            any(not math.isfinite(value) for value in finger_contact_center_offset)):
        raise ValueError("finger_contact_center_offset_m must contain three finite values")
    arm_actuator = selected_scene.get("arm_position_actuator", {})
    if not isinstance(arm_actuator, dict) or set(arm_actuator) - {"kp", "kv", "max_force"}:
        raise ValueError("arm_position_actuator must contain only kp, kv, and max_force")
    arm_kp = float(arm_actuator.get("kp", 300.0))
    arm_kv = float(arm_actuator.get("kv", 30.0))
    arm_max_force = float(arm_actuator.get("max_force", 500.0))
    if any(not math.isfinite(value) or value <= 0.0
           for value in (arm_kp, arm_kv, arm_max_force)):
        raise ValueError("arm_position_actuator kp, kv, and max_force must be positive and finite")
    ground = selected_scene.get("ground", {})
    ET.SubElement(world, "geom", {
        "name": "ground", "type": "plane",
        "size": " ".join(str(value) for value in ground.get("size", [20.0, 20.0, 0.1])),
        "rgba": " ".join(str(value) for value in ground.get("rgba", [0.75, 0.75, 0.75, 1.0])),
        "friction": " ".join(str(value) for value in ground.get("friction", [1.0, 0.005, 0.0001])),
    })
    for obj in selected_scene.get("objects", []):
        attrs = {
            "name": f"{obj['id']}_geom",
            "type": obj["shape"],
            "pos": " ".join(str(value) for value in obj.get("position", [0.0, 0.0, 0.0])),
            "euler": " ".join(str(value) for value in obj.get("rotation", [0.0, 0.0, 0.0])),
            "rgba": " ".join(str(value) for value in obj.get("rgba", [0.6, 0.6, 0.6, 1.0])),
            "friction": " ".join(str(value) for value in obj.get("friction", [1.1, 0.01, 0.001])),
        }
        if obj["shape"] == "box":
            attrs["size"] = " ".join(str(float(value) / 2.0) for value in obj["size"])
        elif obj["shape"] == "sphere":
            attrs["size"] = str(obj["radius"])
        elif obj["shape"] == "cylinder":
            attrs["size"] = f"{obj['radius']} {float(obj['height']) / 2.0}"
        elif obj["shape"] == "mesh":
            mesh_name = f"scene_mesh_{obj['id']}"
            relative_mesh = Path(obj["mesh"])
            if scene_directory is None:
                raise ValueError("scene_directory is required when a scene references a mesh")
            mesh_path = (Path(scene_directory) / relative_mesh).resolve(strict=True)
            if not mesh_path.is_file():
                raise ValueError(f"scene mesh does not exist: {obj['mesh']}")
            ET.SubElement(asset, "mesh", {
                "name": mesh_name,
                "file": str(mesh_path),
                "scale": " ".join(str(value) for value in obj["scale"]),
            })
            attrs.update({"type": "mesh", "mesh": mesh_name})
        if obj.get("motion", "static") == "dynamic":
            body = ET.SubElement(world, "body", {
                "name": f"{obj['id']}_body",
                "pos": attrs.pop("pos"),
                "euler": attrs.pop("euler"),
            })
            ET.SubElement(body, "freejoint", {"name": f"{obj['id']}_free"})
            attrs["pos"] = "0 0 0"
            attrs["euler"] = "0 0 0"
            attrs["mass"] = str(obj["mass_kg"])
            ET.SubElement(body, "geom", attrs)
        else:
            ET.SubElement(world, "geom", attrs)
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
                arm_joint = ja["name"] in ARM_JOINT_NAMES
                position_actuator = {"name": ja["name"], "joint": ja["name"],
                                     "kp": str(arm_kp) if arm_joint else "50",
                                     "kv": str(arm_kv) if arm_joint else "10",
                                     "forcelimited": "true",
                                     "forcerange": (f"{-arm_max_force:g} {arm_max_force:g}"
                                                    if arm_joint else "-300 300")}
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
        for collision_index, collision in enumerate(link.findall("collision")):
            geometry = collision.find("geometry")
            finger_collision = link_name in {"ms42dc_left_finger_link", "ms42dc_right_finger_link"}
            use_contact_box = finger_collision and bool(simplified_finger_collision)
            use_convex_finger = finger_collision and finger_geometry == "convex"
            ga = {"rgba": "0.55 0.58 0.62 1", "mass": "0", "group": "1",
                  "contype": "2" if finger_collision and not use_contact_box else "0",
                  "conaffinity": "1" if finger_collision and not use_contact_box else "0",
                  "friction": "1.1 0.01 0.001"}
            if (finger_collision and flex_pads.get("enabled") and
                    flex_pads.get("style") == "fin_ray" and
                    selected_scene.get("finger_contact_boxes", {}).get(
                        "left" if link_name == "ms42dc_left_finger_link" else "right")):
                # The rigid STL covers the deforming distal finger.  Keep it
                # in the model for mass/geometry conversion but hide its mesh.
                ga["rgba"] = "0.55 0.58 0.62 0"
            if finger_collision and not use_contact_box:
                ga.update(finger_material_attrs)
            ga.update(_pose(collision.find("origin")))
            if use_convex_finger:
                ga.update({"contype": "0", "conaffinity": "0"})
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
                mesh_scale = [float(value) for value in shape.attrib.get("scale", "1 1 1").split()]
                if len(mesh_scale) != 3:
                    raise ValueError(f"mesh scale must contain three values: {uri}")
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
            if use_convex_finger:
                side = "left" if "_left_" in link_name else "right"
                convex_dir = Path(description_dir) / "meshes/gripper/ms42dc/convex"
                manifest = json.loads((convex_dir / "manifest.json").read_text(encoding="utf-8"))
                for index, filename in enumerate(manifest["fingers"][side]["parts"]):
                    convex_name = f"{link_name}_convex_{index}"
                    ET.SubElement(asset, "mesh", {"name": convex_name, "file": str(convex_dir / filename)})
                    convex_attrs = {"name": convex_name, "type": "mesh", "mesh": convex_name,
                                    "rgba": "0 0 0 0", "mass": "0", "group": "3",
                                    "contype": "2", "conaffinity": "1", "friction": "1.1 0.01 0.001"}
                    convex_attrs.update(finger_material_attrs)
                    convex_attrs.update(_pose(collision.find("origin")))
                    ET.SubElement(body, "geom", convex_attrs)
            if use_contact_box:
                contact_attrs = {
                    "name": f"{link_name}_contact_{collision_index}",
                    "type": "box",
                    "size": " ".join(str(value) for value in contact_half_size),
                    "rgba": "0.1 0.8 0.2 0",
                    "mass": "0",
                    "group": "3",
                    "contype": "2",
                    "conaffinity": "1",
                    "friction": "1.1 0.01 0.001",
                }
                contact_attrs.update(finger_material_attrs)
                contact_attrs.update(_pose(collision.find("origin")))
                if shape.tag == "mesh":
                    collision_origin = collision.find("origin")
                    origin_position = ([float(value) for value in collision_origin.get("xyz", "0 0 0").split()]
                                       if collision_origin is not None else [0.0, 0.0, 0.0])
                    mesh_center = _scaled_mesh_bounds_center(path, mesh_scale)
                    contact_attrs["pos"] = " ".join(
                        str(origin_position[index] + mesh_center[index]) for index in range(3))
                contact_position = [float(value) for value in contact_attrs.get("pos", "0 0 0").split()]
                contact_attrs["pos"] = " ".join(
                    str(contact_position[index] + finger_contact_center_offset[index])
                    for index in range(3))
                side = "left" if link_name == "ms42dc_left_finger_link" else "right"
                parts = selected_scene.get("finger_contact_boxes", {}).get(side)
                if parts:
                    for part_index, part in enumerate(parts):
                        attrs = dict(contact_attrs)
                        if flex_pads.get("enabled") and flex_pads.get("style") == "fin_ray" and part_index == len(parts) - 1:
                            attrs.update({"contype": "0", "conaffinity": "0"})
                        elif flex_pads.get("enabled") and flex_pads.get("style") == "fin_ray":
                            attrs["rgba"] = "0.55 0.58 0.62 1"
                            attrs["group"] = "1"
                        attrs["name"] = f"{link_name}_contact_{collision_index}_{part_index}"
                        attrs["pos"] = " ".join(str(float(value)) for value in part["position"])
                        attrs["euler"] = " ".join(str(float(value)) for value in part["rotation"])
                        attrs["size"] = " ".join(str(float(value)) for value in part["half_size"])
                        ET.SubElement(body, "geom", attrs)
                    if flex_pads.get("enabled", False) and flex_pads.get("style") == "fin_ray":
                        add_fin_ray(
                            scene, body, side, parts[-1], finger_material_attrs, flex_pads)
                    elif flex_pads.get("enabled", False):
                        for part_index, part in enumerate(parts):
                            size = [float(flex_pads["size_m"][0]),
                                    2.0 * float(part["half_size"][1]),
                                    2.0 * float(part["half_size"][2])]
                            grid = [int(value) for value in flex_pads["grid"]]
                            yaw = float(part["rotation"][2])
                            direction = 1.0 if side == "left" else -1.0
                            distance = direction * (float(part["half_size"][0]) + size[0] / 2.0)
                            center = [float(part["position"][0]) + math.cos(yaw) * distance,
                                      float(part["position"][1]) + math.sin(yaw) * distance,
                                      float(part["position"][2])]
                            pad = ET.SubElement(body, "flexcomp", {
                                "name": f"ms42dc_{side}_pad_{part_index}", "type": "grid", "dim": "3",
                                "count": " ".join(str(n) for n in grid),
                                "spacing": " ".join(str(size[i] / (grid[i] - 1)) for i in range(3)),
                                "pos": " ".join(str(value) for value in center),
                                "euler": " ".join(str(value) for value in part["rotation"]),
                                "mass": str(flex_pads["mass_kg"]),
                                "radius": str(flex_pads["radius_m"]),
                                "rgba": "0.2 0.75 0.9 1" if side == "left" else "0.95 0.55 0.15 1",
                            })
                            pad_contact = {"contype": "2", "conaffinity": "1", "condim": "4",
                                           "selfcollide": "none", "internal": "false"}
                            pad_contact.update(finger_material_attrs)
                            ET.SubElement(pad, "contact", pad_contact)
                            ET.SubElement(pad, "elasticity", {
                                "young": str(flex_pads["young_pa"]), "poisson": str(flex_pads["poisson"]),
                                "damping": str(flex_pads["damping"]),
                            })
                            pin_x = 0 if side == "left" else grid[0] - 1
                            ET.SubElement(pad, "pin", {"gridrange":
                                f"{pin_x} 0 0 {pin_x} {grid[1] - 1} {grid[2] - 1}"})
                else:
                    ET.SubElement(body, "geom", contact_attrs)
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
