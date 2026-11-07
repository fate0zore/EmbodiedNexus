#!/usr/bin/env python3
"""Standalone MuJoCo 3.2.7 compliant two-finger grasp experiment."""

import argparse
import json
import time
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def add_finger(parent, side):
    """Add a tapered two-wall ribbed flex finger to one moving jaw.

    Args:
        parent: The moving MuJoCo body element.
        side: Finger side name used for diagnostics.

    Returns:
        None.
    """
    points = []
    triangles = []
    levels = 7
    for i in range(levels):
        fraction = i / (levels - 1)
        width = 0.018 - 0.010 * fraction
        z = -0.096 * fraction
        for x, dz in ((-width, 0.0015), (-width, -0.0015),
                      (width, 0.0015), (width, -0.0015)):
            points.append((x, 0.0, z + dz))
        base = i * 4
        triangles.extend(((base, base + 1, base + 3), (base, base + 3, base + 2)))
        if i:
            prev = base - 4
            for a, b in ((0, 1), (2, 3)):
                triangles.extend(((prev + a, base + a, base + b),
                                  (prev + a, base + b, prev + b)))
    flex = ET.SubElement(parent, "flexcomp", {
        "name": side + "_fin", "type": "direct", "dim": "2", "radius": "0.003",
        "mass": "0.04", "rgba": "0.08 0.75 0.8 1",
        "point": " ".join(f"{v:.7g}" for point in points for v in point),
        "element": " ".join(str(v) for triangle in triangles for v in triangle),
    })
    ET.SubElement(flex, "edge", {"equality": "true", "solref": "0.001 1",
                                  "solimp": "0.99 0.999 0.0001 0.5 2"})
    ET.SubElement(flex, "contact", {"selfcollide": "none", "internal": "false",
                                     "contype": "2", "conaffinity": "1",
                                     "condim": "4", "friction": "5 0.02 0.001"})
    ET.SubElement(flex, "pin", {"id": "0 1 2 3"})


def make_model():
    """Generate a standalone two-finger scene and a horizontal cylinder.

    Returns:
        MuJoCo model compiled from the generated MJCF.
    """
    root = ET.Element("mujoco", {"model": "fin_ray_validation"})
    ET.SubElement(root, "option", {"timestep": "0.0005", "gravity": "0 0 -9.8",
                                    "integrator": "implicitfast", "cone": "elliptic"})
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", {"pos": "0 0 1"})
    ET.SubElement(world, "geom", {"name": "ground", "type": "plane", "size": "0.5 0.5 0.05",
                                  "rgba": "0.35 0.35 0.35 1", "friction": "1 0.02 0.001"})
    obj = ET.SubElement(world, "body", {"name": "cylinder", "pos": "0 0 0.025"})
    ET.SubElement(obj, "freejoint", {"name": "cylinder_free"})
    ET.SubElement(obj, "geom", {"name": "cylinder_geom", "type": "cylinder", "size": "0.025 0.05",
                                "quat": "0.7071067812 0.7071067812 0 0", "mass": "0.1",
                                "rgba": "0.1 0.8 0.1 1", "friction": "5 0.02 0.001",
                                "condim": "4", "priority": "1"})
    hand = ET.SubElement(world, "body", {"name": "hand", "pos": "0 0 0.115", "gravcomp": "1"})
    ET.SubElement(hand, "joint", {"name": "lift", "type": "slide", "axis": "0 0 1", "damping": "3"})
    ET.SubElement(hand, "geom", {"type": "box", "size": "0.09 0.015 0.008", "mass": "0.2",
                                 "contype": "0", "conaffinity": "0", "rgba": "0.3 0.3 0.3 1"})
    for side, sign in (("left", -1), ("right", 1)):
        jaw = ET.SubElement(hand, "body", {"name": side + "_jaw", "pos": f"{sign * 0.059} 0 0",
                                         "gravcomp": "1"})
        ET.SubElement(jaw, "joint", {"name": side + "_slide", "type": "slide",
                                     "axis": f"{-sign} 0 0", "range": "0 0.04", "damping": "2"})
        ET.SubElement(jaw, "geom", {"type": "box", "size": "0.02 0.006 0.004", "mass": "0.05",
                                    "contype": "0", "conaffinity": "0", "rgba": "0.3 0.3 0.3 1"})
        add_finger(jaw, side)
    actuator = ET.SubElement(root, "actuator")
    ET.SubElement(actuator, "position", {"name": "lift", "joint": "lift", "kp": "600", "kv": "35"})
    for side in ("left", "right"):
        ET.SubElement(actuator, "position", {"name": side, "joint": side + "_slide", "kp": "500", "kv": "25",
                                             "ctrllimited": "true", "ctrlrange": "0 0.04"})
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    # MuJoCo 3.2.7 creates three slide joints per unpinned flex vertex.
    # Restore each vertex towards its finger plane while leaving in-plane motion free.
    for side in ("left", "right"):
        flex_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, side + "_fin")
        first = int(model.flex_vertadr[flex_id])
        count = int(model.flex_vertnum[flex_id])
        for body_id in model.flex_vertbodyid[first:first + count]:
            body_id = int(body_id)
            for joint_id in range(int(model.body_jntadr[body_id]),
                                  int(model.body_jntadr[body_id] + model.body_jntnum[body_id])):
                if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_SLIDE and abs(model.jnt_axis[joint_id, 1]) > 0.9:
                    model.jnt_stiffness[joint_id] = 100.0
                    model.dof_damping[model.jnt_dofadr[joint_id]] = 0.1
    return model


def measure(model, data, rest):
    """Read target position, bilateral contacts and finger deformation.

    Args:
        model: Compiled MuJoCo model.
        data: Current simulation state.
        rest: Finger vertices in their jaw frames at rest.

    Returns:
        JSON-serializable measurement mapping.
    """
    target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cylinder_geom")
    contacts = {"left": {"count": 0, "normal_force_n": 0.0},
                "right": {"count": 0, "normal_force_n": 0.0}}
    for i in range(data.ncon):
        contact = data.contact[i]
        for a, b in ((0, 1), (1, 0)):
            if int(contact.geom[b]) != target or int(contact.flex[a]) < 0:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_FLEX, int(contact.flex[a]))
            side = name.split("_")[0]
            if side in contacts:
                force = np.zeros(6)
                mujoco.mj_contactForce(model, data, i, force)
                contacts[side]["count"] += 1
                contacts[side]["normal_force_n"] += float(force[0])
    deformation = {}
    for side in contacts:
        flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, side + "_fin")
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, side + "_jaw")
        start, count = int(model.flex_vertadr[flex]), int(model.flex_vertnum[flex])
        vertices = (data.flexvert_xpos[start:start + count] - data.xpos[body]) @ data.xmat[body].reshape(3, 3)
        deformation[side] = float(np.max(np.linalg.norm(vertices - rest[side], axis=1)))
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cylinder")
    return {"time_s": float(data.time), "cylinder_position_m": data.xpos[body].tolist(),
            "contacts": contacts, "max_displacement_m": deformation}


def run(gui, output):
    """Run settling, closure, lifting and a hold and optionally show the viewer.

    Args:
        gui: Whether to open the native MuJoCo viewer in real time.
        output: Optional output JSON file path.

    Returns:
        Summary dictionary containing measured outcomes and samples.
    """
    model = make_model()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    rest = {}
    for side in ("left", "right"):
        flex = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_FLEX, side + "_fin")
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, side + "_jaw")
        start, count = int(model.flex_vertadr[flex]), int(model.flex_vertnum[flex])
        rest[side] = (data.flexvert_xpos[start:start + count] - data.xpos[body]) @ data.xmat[body].reshape(3, 3)
    viewer = None
    if gui:
        from mujoco import viewer as viewer_module
        viewer = viewer_module.launch_passive(model, data)
    samples = []
    start_wall = time.monotonic()
    total_steps = round(8.0 / model.opt.timestep)
    try:
        for step in range(total_steps):
            t = step * model.opt.timestep
            close = 0.040 * min(max((t - 0.5) / 2.5, 0.0), 1.0)
            lift = 0.10 * min(max((t - 3.5) / 2.0, 0.0), 1.0)
            data.ctrl[:] = (lift, close, close)
            mujoco.mj_step(model, data)
            if any(warning.number for warning in data.warning):
                raise RuntimeError(f"MuJoCo numerical warning at simulation time {data.time:.4f}")
            if step % 100 == 0:
                samples.append(measure(model, data, rest))
            if viewer and step % 40 == 0:
                viewer.sync()
                delay = data.time - (time.monotonic() - start_wall)
                if delay > 0:
                    time.sleep(delay)
    finally:
        if viewer:
            viewer.close()
    initial = samples[0]["cylinder_position_m"][2]
    hold = [sample for sample in samples if sample["time_s"] >= 5.5]
    result = {"mujoco_version": mujoco.__version__, "initial_height_m": initial,
              "maximum_height_m": max(s["cylinder_position_m"][2] for s in samples),
              "hold_min_lift_m": min(s["cylinder_position_m"][2] - initial for s in hold),
              "hold_duration_s": hold[-1]["time_s"] - hold[0]["time_s"],
              "lift_5cm_for_2s": (hold[-1]["time_s"] - hold[0]["time_s"] >= 2.0 and
                                    all(s["cylinder_position_m"][2] - initial >= 0.05 for s in hold)),
              "bilateral_contact_samples": sum(all(s["contacts"][side]["count"] > 0 for side in ("left", "right")) for s in samples),
              "peak_normal_force_n": {side: max(s["contacts"][side]["normal_force_n"] for s in samples) for side in ("left", "right")},
              "max_finger_displacement_m": {side: max(s["max_displacement_m"][side] for s in samples) for side in ("left", "right")},
              "samples": samples}
    if output:
        with open(output, "w", encoding="utf-8") as stream:
            json.dump(result, stream, indent=2)
    return result


def main():
    """Parse arguments, run the experiment, and print its measured summary.

    Returns:
        None.
    """
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="display the native MuJoCo viewer")
    parser.add_argument("--output", help="save the time series to a JSON file")
    args = parser.parse_args()
    result = run(args.gui, args.output)
    print(json.dumps({key: value for key, value in result.items() if key != "samples"}, indent=2))


if __name__ == "__main__":
    main()
