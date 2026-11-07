"""Tests for bilateral simulated gripper contact reporting."""

import unittest

import mujoco

from embodied_nexus_sim_mujoco.simulator import _bilateral_finger_contacts, _finger_visualization


class BilateralFingerContactTests(unittest.TestCase):
    """Check that a single finger cannot report a grasped object."""

    def _model(self, right_finger_x: float):
        """Compile a tiny scene with one dynamic object and two finger bodies.

        Args:
            right_finger_x: X position of the right finger relative to the object.

        Returns:
            Compiled MuJoCo model and initialized data.
        """
        xml = f"""<mujoco>
          <worldbody>
            <body name="ms42dc_left_finger_link" pos="-0.02 0 0">
              <geom name="left_geom" type="box" size="0.04 0.04 0.04"/>
            </body>
            <body name="ms42dc_right_finger_link" pos="{right_finger_x} 0 0">
              <geom name="right_geom" type="box" size="0.04 0.04 0.04"/>
            </body>
            <body name="target_box_body" pos="0 0 0">
              <freejoint/>
              <geom name="target_box_geom" type="box" size="0.03 0.03 0.03"/>
            </body>
          </worldbody>
        </mujoco>"""
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        return model, data

    def test_one_finger_contact_does_not_report_grasp(self):
        """Return no target when only the left finger touches it.

        Returns:
            None.
        """
        model, data = self._model(right_finger_x=1.0)
        self.assertGreater(data.ncon, 0)
        self.assertEqual(_bilateral_finger_contacts(model, data, {"target_box"}), set())

    def test_both_fingers_contact_reports_target(self):
        """Report a target only when each finger touches the same object.

        Returns:
            None.
        """
        model, data = self._model(right_finger_x=0.02)
        self.assertGreaterEqual(data.ncon, 2)
        self.assertEqual(_bilateral_finger_contacts(model, data, {"target_box"}), {"target_box"})

    def test_visualization_reports_finger_boxes_and_local_contacts(self):
        """Report collision dimensions and contact positions in the finger frame.

        Returns:
            None.
        """
        model, data = self._model(right_finger_x=1.0)
        fingers = {item["link"]: item for item in _finger_visualization(model, data)["fingers"]}
        left = fingers["ms42dc_left_finger_link"]
        self.assertEqual(left["boxes"][0]["size"], [0.08, 0.08, 0.08])
        self.assertTrue(any(item["other"] == "target_box_geom" for item in left["contacts"]))
        self.assertFalse(fingers["ms42dc_right_finger_link"]["contacts"])

    def test_flexible_contacts_report_both_sides(self):
        """Recognize flex contacts without geom IDs and publish both pad surfaces.

        Returns:
            None.
        """
        xml = '''<mujoco><worldbody>
          <body name="ms42dc_left_finger_link" pos="-0.02 0 0">
            <flexcomp name="ms42dc_left_pad" type="grid" dim="3" count="2 2 2"
              spacing="0.01 0.01 0.01" mass="0.01" radius="0.001">
              <pin gridrange="0 0 0 0 1 1"/><contact selfcollide="none"/>
            </flexcomp>
          </body>
          <body name="ms42dc_right_finger_link" pos="0.02 0 0">
            <flexcomp name="ms42dc_right_pad" type="grid" dim="3" count="2 2 2"
              spacing="0.01 0.01 0.01" mass="0.01" radius="0.001">
              <pin gridrange="1 0 0 1 1 1"/><contact selfcollide="none"/>
            </flexcomp>
          </body>
          <body name="target"><freejoint name="target_free"/><geom name="target_box_geom" type="box"
            size="0.017 0.02 0.02"/></body>
        </worldbody></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        self.assertTrue(any(-1 in contact.geom and max(contact.flex) >= 0
                            for contact in data.contact[:data.ncon]))
        self.assertEqual(_bilateral_finger_contacts(model, data, {"target_box"}), {"target_box"})
        fingers = _finger_visualization(model, data)["fingers"]
        self.assertTrue(all(finger["pads"] and finger["contacts"] for finger in fingers))
        self.assertTrue(all(max(max(face) for face in finger["pads"][0]["triangles"]) <
                            len(finger["pads"][0]["vertices"]) for finger in fingers))
        target = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "target_free")
        data.qpos[model.jnt_qposadr[target]] = -0.02
        mujoco.mj_forward(model, data)
        self.assertEqual(_bilateral_finger_contacts(model, data, {"target_box"}), set())
        fingers = _finger_visualization(model, data)["fingers"]
        self.assertTrue(fingers[0]["contacts"])
        self.assertFalse(fingers[1]["contacts"])
        data.qpos[model.jnt_qposadr[target]] = 0.02
        mujoco.mj_forward(model, data)
        self.assertEqual(_bilateral_finger_contacts(model, data, {"target_box"}), set())
        fingers = _finger_visualization(model, data)["fingers"]
        self.assertFalse(fingers[0]["contacts"])
        self.assertTrue(fingers[1]["contacts"])


if __name__ == "__main__":
    unittest.main()
