from __future__ import annotations

import asyncio
import importlib.util
import unittest


try:
    from embodied_nexus_arm.web_server import create_app
except ModuleNotFoundError:
    create_app = None


class FakeGateway:
    device_id = "arm_main"

    def status(self):
        return {"connected": True, "feedback_valid": True}


@unittest.skipIf(create_app is None, "Starlette is not installed in this local Python environment")
class ArmWebApiTests(unittest.TestCase):
    @staticmethod
    def request(app, path, authorization=b""):
        async def run():
            sent = []
            receive_events = [{"type": "http.request", "body": b"", "more_body": False}]

            async def receive():
                return receive_events.pop(0)

            async def send(message):
                sent.append(message)

            scope = {"type": "http", "asgi": {"version": "3.0", "spec_version": "2.3"},
                     "http_version": "1.1", "method": "GET", "scheme": "http",
                     "path": path, "raw_path": path.encode(), "query_string": b"",
                     "headers": [(b"authorization", authorization)], "client": ("127.0.0.1", 1),
                     "server": ("127.0.0.1", 80), "root_path": ""}
            await app(scope, receive, send)
            return sent
        return asyncio.run(run())

    def test_http_requires_bearer_and_returns_state(self):
        app = create_app(FakeGateway(), "token-123")
        route = "/api/v1/arms/arm_main/status"
        unauthorized = self.request(app, route)
        self.assertEqual(unauthorized[0]["status"], 401)
        authorized = self.request(app, route, b"Bearer token-123")
        self.assertEqual(authorized[0]["status"], 200)
