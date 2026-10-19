from __future__ import annotations

import unittest
from unittest.mock import patch

from embodied_nexus_client import CoreClient


class CoreSdkTests(unittest.TestCase):
    def test_accessors_share_the_core_http_address_and_route_streams_by_device(self):
        client = CoreClient("http://robot:8780", "token")
        base = client.base("base_lab")
        arm = client.arm("arm_lab")
        gripper = client.gripper("gripper_lab")
        camera = client.camera("camera_lab")
        lidar = client.lidar("lidar_lab")
        self.assertEqual(base.base_url, "http://robot:8780")
        self.assertEqual(base._prefix, "/api/v1/bases/base_lab")
        self.assertEqual(base.websocket_path, "/api/v1/bases/base_lab/ws")
        self.assertEqual(arm.websocket_path, "/api/v1/arms/arm_lab/ws")
        self.assertEqual(gripper._prefix, "/api/v1/grippers/gripper_lab")
        self.assertEqual(camera.websocket_path, "/api/v1/cameras/camera_lab/ws")
        self.assertEqual(lidar.websocket_path, "/api/v1/lidars/lidar_lab/ws")

    def test_management_methods_use_core_paths(self):
        client = CoreClient("http://robot:8780", "token")
        with patch.object(client, "_request", return_value={"success": True}) as request:
            client.start("arm_main")
            self.assertEqual(request.call_args.args[:2],
                             ("POST", "/api/v1/core/devices/arm_main"))
            self.assertEqual(request.call_args.args[2], {"action": "start"})
            self.assertEqual(request.call_args.kwargs["timeout"], client.management_timeout)
            client.stop_all()
            self.assertEqual(request.call_args.args[:2],
                             ("POST", "/api/v1/core/devices/manage_all"))
            self.assertEqual(request.call_args.args[2], {"action": "stop_all"})
            self.assertEqual(request.call_args.kwargs["timeout"], client.management_timeout)


if __name__ == "__main__":
    unittest.main()
