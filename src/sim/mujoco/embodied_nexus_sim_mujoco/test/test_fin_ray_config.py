"""Exercise Fin Ray configuration against the compiled MuJoCo model."""

import copy
import shutil
import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np

from embodied_nexus_sim_mujoco.fin_ray import add_fin_ray, configure_fin_ray


class FinRayConfigTest(unittest.TestCase):
    """Check generated geometry, material and constraints."""

    def make_model(self, changes=None, contact_object=False):
        """Compile one configured finger.

        Args:
            changes: Optional scene parameter overrides.
            contact_object: Whether to place a sphere against the middle rib.

        Returns:
            A compiled MuJoCo model.
        """
        pads = {"size_m": [0.003, 0.045, 0.014], "grid": [2, 7, 3],
                "mass_kg": 0.01, "young_pa": 1000.0, "poisson": 0.3,
                "damping": 0.00001, "radius_m": 0.0002,
                "fin_ray_edge_time_constant_s": 0.0005}
        pads.update(changes or {})
        root = ET.Element("mujoco")
        ET.SubElement(root, "option", {"timestep": "0.00025", "gravity": "0 0 0",
                                        "integrator": "implicitfast", "solver": "CG", "iterations": "50"})
        world = ET.SubElement(root, "worldbody")
        if contact_object:
            ET.SubElement(world, "geom", {"name": "target", "type": "sphere",
                                           "size": "0.003", "pos": "0 -0.010255 0.009"})
        body = ET.SubElement(world, "body", {"name": "finger"})
        part = {"position": [0, 0, 0], "rotation": [0, 0, 0],
                "half_size": [0.009478, 0.032755, 0.008627]}
        add_fin_ray(root, body, "left", part, {}, pads)
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        configure_fin_ray(model, 500.0)
        return model

    def test_geometry_mass_and_constraint_follow_configuration(self):
        """Apply dimensions, resolution, mass, radius and edge time constant.

        Returns:
            None.
        """
        model = self.make_model()
        self.assertEqual(model.flex_dim[0], 3)
        self.assertEqual(model.flex_vertnum[0], 168)
        self.assertAlmostEqual(model.flex_radius[0], 0.0002)
        self.assertAlmostEqual(model.eq_solref[0, 0], 0.0005)
        body_ids = np.unique(model.flex_vertbodyid)
        moving = body_ids[body_ids > 1]
        self.assertGreater(len(moving), 0)
        self.assertAlmostEqual(model.body_mass[moving[0]], 0.01 / 168)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        vertices = data.flexvert_xpos.copy()
        self.assertAlmostEqual(np.ptp(vertices[:, 1]), 0.045)
        self.assertAlmostEqual(np.ptp(vertices[:, 2]), 0.014)
        changed = self.make_model({"grid": [3, 8, 4], "size_m": [0.002, 0.05, 0.02],
                                   "mass_kg": 0.02, "radius_m": 0.0004,
                                   "fin_ray_edge_time_constant_s": 0.004})
        self.assertEqual(changed.flex_vertnum[0], 3 * 2 * 8 * 2 * 4)
        self.assertAlmostEqual(changed.flex_radius[0], 0.0004)
        self.assertAlmostEqual(changed.eq_solref[0, 0], 0.004)
        changed_data = mujoco.MjData(changed)
        mujoco.mj_forward(changed, changed_data)
        self.assertAlmostEqual(np.ptp(changed_data.flexvert_xpos[:, 1]), 0.05)
        self.assertAlmostEqual(np.ptp(changed_data.flexvert_xpos[:, 2]), 0.02)
        self.assertTrue(np.any(np.isclose(model.jnt_stiffness, 500.0)))
        configure_fin_ray(model, 1200.0)
        self.assertTrue(np.any(np.isclose(model.jnt_stiffness, 1200.0)))

    def test_elastic_material_changes_passive_forces(self):
        """Show that changing material coefficients changes simulated forces.

        Returns:
            None.
        """
        forces = []
        for overrides in ({}, {"young_pa": 2000.0}, {"poisson": 0.45}, {"damping": 0.001}):
            model = self.make_model(overrides)
            data = mujoco.MjData(model)
            data.qpos[-3] = 0.0001
            data.qvel[-3] = 0.1
            mujoco.mj_forward(model, data)
            forces.append(copy.copy(data.qfrc_passive))
        for force in forces[1:]:
            self.assertGreater(np.linalg.norm(force - forces[0]), 1e-10)

    def test_volume_finger_contacts_target(self):
        """Detect actual contact between the generated flex and a sphere.

        Returns:
            None.
        """
        for changes, timestep in (({}, 0.00025),
                                  ({"grid": [2, 3, 2], "fin_ray_edge_time_constant_s": 0.001}, 0.0005)):
            with self.subTest(changes=changes):
                model = self.make_model(changes, contact_object=True)
                model.opt.timestep = timestep
                data = mujoco.MjData(model)
                mujoco.mj_forward(model, data)
                target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "target")
                self.assertTrue(any(0 in contact.flex and target in contact.geom for contact in data.contact))
                for _ in range(round(0.1 / timestep)):
                    mujoco.mj_step(model, data)
                self.assertAlmostEqual(data.time, 0.1)
                self.assertTrue(all(warning.number == 0 for warning in data.warning))

    def test_batched_steps_match_individual_steps(self):
        """Keep the same physics when substeps are issued in one call.

        Returns:
            None.
        """
        model = self.make_model({"grid": [2, 3, 2], "fin_ray_edge_time_constant_s": 0.001},
                                contact_object=True)
        individual, batched = mujoco.MjData(model), mujoco.MjData(model)
        for _ in range(20):
            mujoco.mj_step(model, individual)
        mujoco.mj_step(model, batched, nstep=20)
        np.testing.assert_allclose(individual.qpos, batched.qpos, rtol=1e-12, atol=1e-12)
        np.testing.assert_allclose(individual.qvel, batched.qvel, rtol=1e-12, atol=1e-12)

    def test_edge_time_constant_changes_loaded_motion(self):
        """Verify the edge constraint affects deformation under the same load.

        Returns:
            None.
        """
        deflections = []
        for value in (0.0005, 0.004):
            model = self.make_model({"fin_ray_edge_time_constant_s": value})
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            tip = int(model.flex_vertbodyid[-1])
            initial = data.xpos[tip].copy()
            data.xfrc_applied[tip, 0] = 0.001
            for _ in range(400):
                mujoco.mj_step(model, data)
            self.assertAlmostEqual(data.time, 0.1)
            deflections.append(np.linalg.norm(data.xpos[tip] - initial))
        self.assertGreater(deflections[1], deflections[0] + 1e-6)

    def test_rest_state_is_numerically_stable(self):
        """Step the configured elastic finger without numerical warnings.

        Returns:
            None.
        """
        model = self.make_model()
        data = mujoco.MjData(model)
        data.qpos[-3] = 0.0001
        for _ in range(200):
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(data.time, 0.05)
        self.assertTrue(np.all(np.isfinite(data.qpos)))
        self.assertTrue(all(warning.number == 0 for warning in data.warning))

    @unittest.skipUnless(shutil.which("xacro"), "ROS xacro is required")
    def test_green_cylinder_scene_compiles_and_steps(self):
        """Compile and step both configured pads on the Arachne model.

        Returns:
            None.
        """
        import yaml
        from embodied_nexus_sim_mujoco.model import make_scene

        package = Path(__file__).resolve().parents[1]
        description = package.parents[1] / "common/third_party/embodied_nexus_sim_description"
        scene_dir = package / "config/scenes"
        scene = yaml.safe_load((scene_dir / "scene-1793463913737.yaml").read_text(encoding="utf-8"))
        scene["finger_collision_geometry"] = "boxes"
        scene["finger_flex_pads"]["enabled"] = True
        urdf = subprocess.run(["xacro", str(description / "urdf/arachne.urdf.xacro"),
                               "gazebo_drive_wheels:=true", "with_mimic_joints:=false"],
                              check=True, capture_output=True, text=True).stdout
        model = mujoco.MjModel.from_xml_string(make_scene(urdf, description, scene_config=scene,
                                                          scene_directory=scene_dir))
        self.assertAlmostEqual(model.opt.timestep, 0.0005)
        self.assertEqual(model.opt.solver, mujoco.mjtSolver.mjSOL_CG)
        self.assertEqual(model.opt.iterations, 50)
        configure_fin_ray(model, scene["finger_flex_pads"]["fin_ray_out_of_plane_stiffness_n_m"])
        data = mujoco.MjData(model)
        for name, value in scene["initial_arm_joint_positions_rad"].items():
            joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            data.qpos[model.jnt_qposadr[joint]] = value
            data.ctrl[actuator] = value
        self.assertEqual(model.nflex, 2)
        for _ in range(round(0.5 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        self.assertAlmostEqual(data.time, 0.5)
        self.assertTrue(np.all(np.isfinite(data.qpos)))
        self.assertTrue(all(warning.number == 0 for warning in data.warning))


if __name__ == "__main__":
    unittest.main()
