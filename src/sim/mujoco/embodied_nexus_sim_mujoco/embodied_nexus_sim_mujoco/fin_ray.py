"""Compliant ribbed finger geometry for the MuJoCo 3.2 simulation."""

import xml.etree.ElementTree as ET

import mujoco


def add_fin_ray(scene, body, side, part, material, pads):
    """Attach a tapered, ribbed flexible finger to a robot finger body.

    Args:
        scene: Root MuJoCo XML element for the edge constraint.
        body: MuJoCo finger body XML element.
        side: Finger side, left or right.
        part: Distal collision-box pose and half-size mapping.
        material: MuJoCo contact material attributes.
        pads: Validated finger_flex_pads scene configuration.

    Returns:
        None.
    """
    import numpy as np

    thickness, length, depth = (float(value) for value in pads["size_m"])
    across, levels, layers = (int(value) for value in pads["grid"])
    width = float(part["half_size"][0])
    if thickness >= width / 2 or thickness >= length / levels:
        raise ValueError("Fin Ray rib thickness must fit the finger width and spacing")

    # Each station has a transverse rib, and the outer strips connect stations.
    # Extruding this lattice creates tetrahedra with a shared, conforming surface.
    plane, triangles = [], []
    columns = 2 * across
    for i in range(levels):
        fraction = i / (levels - 1)
        half_width = width * (1.0 - 0.5 * fraction)
        xs = ([-half_width + thickness * j / (across - 1) for j in range(across)] +
              [half_width - thickness + thickness * j / (across - 1) for j in range(across)])
        center_y = thickness / 2 + (length - thickness) * fraction
        for y in (center_y - thickness / 2, center_y + thickness / 2):
            plane.extend((x, y) for x in xs)
    for row in range(2 * levels - 1):
        spans = range(columns - 1) if row % 2 == 0 else (
            list(range(across - 1)) + list(range(across, columns - 1)))
        for col in spans:
            a, b = row * columns + col, (row + 1) * columns + col
            triangles.extend(((a, a + 1, b + 1), (a, b + 1, b)))
    points = [(x, y, depth * (layer / (layers - 1) - 0.5))
              for layer in range(layers) for x, y in plane]
    elements = []
    for layer in range(layers - 1):
        lower, upper = layer * len(plane), (layer + 1) * len(plane)
        for triangle in triangles:
            a, b, c = sorted(triangle)
            for tet in ((lower + a, lower + b, lower + c, upper + c),
                        (lower + a, lower + b, upper + b, upper + c),
                        (lower + a, upper + a, upper + b, upper + c)):
                v = [np.asarray(points[index]) for index in tet]
                if np.linalg.det(np.stack((v[1] - v[0], v[2] - v[0], v[3] - v[0]))) < 0:
                    tet = (tet[0], tet[2], tet[1], tet[3])
                elements.append(tet)
    import math
    yaw = float(part["rotation"][2])
    center = part["position"]
    box_length = 2.0 * float(part["half_size"][1])
    root = [float(center[0]) + math.sin(yaw) * box_length / 2,
            float(center[1]) - math.cos(yaw) * box_length / 2, float(center[2])]
    flex = ET.SubElement(body, "flexcomp", {
        "name": f"ms42dc_{side}_pad_fin", "type": "direct", "dim": "3", "radius": str(pads["radius_m"]),
        "mass": str(pads["mass_kg"]), "rgba": "0.05 0.75 0.8 1", "pos": " ".join(map(str, root)),
        "euler": " ".join(map(str, part["rotation"])),
        "point": " ".join(f"{v:.8g}" for point in points for v in point),
        "element": " ".join(str(v) for tet in elements for v in tet),
    })
    ET.SubElement(flex, "elasticity", {"young": str(pads["young_pa"]),
                                        "poisson": str(pads["poisson"]),
                                        "damping": str(pads["damping"])})
    equality = scene.find("equality")
    if equality is None:
        equality = ET.SubElement(scene, "equality")
    ET.SubElement(equality, "flex", {"flex": f"ms42dc_{side}_pad_fin",
                                     "solref": f"{pads.get('fin_ray_edge_time_constant_s', 0.0005)} 1",
                                     "solimp": "0.99 0.999 0.0001 0.5 2"})
    attrs = {"selfcollide": "none", "internal": "false", "contype": "2", "conaffinity": "1", "condim": "4"}
    attrs.update(material)
    ET.SubElement(flex, "contact", attrs)
    pinned = [layer * len(plane) + point for layer in range(layers) for point in range(2 * columns)]
    ET.SubElement(flex, "pin", {"id": " ".join(map(str, pinned))})


def configure_fin_ray(model, out_of_plane_stiffness_n_m=500.0):
    """Add out-of-plane springs to the generated flexible-node joints.

    Args:
        model: Compiled MuJoCo model, modified in place.
        out_of_plane_stiffness_n_m: Plane-restoring spring stiffness in N/m.

    Returns:
        None.
    """
    for side in ("left", "right"):
        flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, f"ms42dc_{side}_pad_fin")
        if flex < 0:
            continue
        first, count = int(model.flex_vertadr[flex]), int(model.flex_vertnum[flex])
        for body in model.flex_vertbodyid[first:first + count]:
            body = int(body)
            start, size = int(model.body_jntadr[body]), int(model.body_jntnum[body])
            for joint in range(start, start + size):
                if model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_SLIDE and abs(model.jnt_axis[joint, 2]) > 0.9:
                    model.jnt_stiffness[joint] = out_of_plane_stiffness_n_m
                    model.dof_damping[model.jnt_dofadr[joint]] = 0.1
