from __future__ import annotations

import json
import socket
import threading
import time
import unittest
from pathlib import Path

from embodied_nexus_arm.backend import AuboBackend, AuboJsonRpc, _six
from embodied_nexus_arm.config import load_config


def _read_request(conn):
    data = bytearray()
    decoder = json.JSONDecoder()
    while True:
        chunk = conn.recv(1)
        if not chunk:
            raise ConnectionError("client closed before sending a request")
        data.extend(chunk)
        try:
            request, _ = decoder.raw_decode(data.decode("utf-8"))
            return request
        except json.JSONDecodeError:
            continue


def _send_response(conn, request, result=None, error=None):
    response = {"jsonrpc": "2.0", "id": request["id"]}
    response["error" if error is not None else "result"] = error if error is not None else result
    conn.sendall(json.dumps(response, ensure_ascii=False).encode("utf-8"))


class RpcTests(unittest.TestCase):
    def test_fragmented_and_sequential_json_rpc_responses(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]

        def serve():
            conn, _ = server.accept()
            with conn:
                for result in (["rob1"], {"note": "机械臂", "joints": [1, 2, 3, 4, 5, 6]}):
                    request = _read_request(conn)
                    reply = json.dumps({"jsonrpc": "2.0", "id": request["id"],
                                        "result": result}, ensure_ascii=False).encode()
                    if isinstance(result, dict):
                        split = reply.index("机".encode("utf-8")) + 1
                        chunks = (reply[:split], reply[split:split + 1], reply[split + 1:])
                    else:
                        chunks = (reply[:5], reply[5:13], reply[13:])
                    for chunk in chunks:
                        conn.sendall(chunk)

        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        rpc = AuboJsonRpc("127.0.0.1", port, 1.0)
        try:
            rpc.connect()
            self.assertEqual(rpc.robot_name, "rob1")
            self.assertEqual(rpc.call("read"), {"note": "机械臂", "joints": [1, 2, 3, 4, 5, 6]})
        finally:
            rpc.close()
            server.close()
            worker.join(timeout=1.0)

    def test_json_rpc_errors_and_timeouts(self):
        error_server = socket.socket()
        error_server.bind(("127.0.0.1", 0))
        error_server.listen(1)
        error_port = error_server.getsockname()[1]

        def serve_error():
            conn, _ = error_server.accept()
            with conn:
                first = _read_request(conn)
                _send_response(conn, first, result=["rob1"])
                second = _read_request(conn)
                _send_response(conn, second, error={"code": -1, "message": "rejected"})

        worker = threading.Thread(target=serve_error, daemon=True)
        worker.start()
        rpc = AuboJsonRpc("127.0.0.1", error_port, 1.0)
        try:
            rpc.connect()
            with self.assertRaisesRegex(RuntimeError, "read failed:.*rejected"):
                rpc.call("read")
        finally:
            rpc.close()
            error_server.close()
            worker.join(timeout=1.0)

        timeout_server = socket.socket()
        timeout_server.bind(("127.0.0.1", 0))
        timeout_server.listen(1)
        timeout_port = timeout_server.getsockname()[1]

        def serve_timeout():
            conn, _ = timeout_server.accept()
            with conn:
                _send_response(conn, _read_request(conn), result=["rob1"])
                time.sleep(0.15)

        worker = threading.Thread(target=serve_timeout, daemon=True)
        worker.start()
        rpc = AuboJsonRpc("127.0.0.1", timeout_port, 0.04)
        try:
            rpc.connect()
            with self.assertRaises(TimeoutError):
                rpc.call("slow_read")
            self.assertIsNone(rpc._socket)
        finally:
            rpc.close()
            timeout_server.close()
            worker.join(timeout=1.0)

    def test_aubo_backend_reads_state_using_a_fake_rpc_controller(self):
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        port = server.getsockname()[1]
        min_positions = [-6.0, -3.0, -3.0, -3.0, -3.0, -6.0]
        max_positions = [6.0, 3.0, 3.0, 3.0, 3.0, 6.0]
        values = {
            "getRobotNames": ["rob1"],
            "rob1.RobotConfig.getJointMinPositions": min_positions,
            "rob1.RobotConfig.getJointMaxPositions": max_positions,
            "rob1.RobotConfig.getPayload": [0.8, [0.0, 0.0, 0.1], [0.0] * 3, [0.0] * 6],
            "rob1.RobotConfig.getTcpOffset": [0.0] * 6,
            "rob1.RobotState.getJointPositions": [0.1] * 6,
            "rob1.RobotState.getJointSpeeds": [0.0] * 6,
            "rob1.RobotState.getRobotModeType": "Running",
            "rob1.RobotState.getSafetyModeType": "Normal",
            "rob1.RobotManage.isFreedriveEnabled": False,
        }
        worker_errors = []

        def serve():
            try:
                conn, _ = server.accept()
                with conn:
                    for _ in range(len(values)):
                        request = _read_request(conn)
                        _send_response(conn, request, result=values[request["method"]])
            except Exception as exc:
                worker_errors.append(exc)

        worker = threading.Thread(target=serve, daemon=True)
        worker.start()
        config_path = Path(__file__).resolve().parents[1] / "config" / "aubo_i5_real.yaml"
        config = load_config(config_path)
        config["rpc"].update(host="127.0.0.1", port=port, timeout_sec=1.0)
        backend = AuboBackend(config)
        try:
            backend.open()
            state = backend.poll()
            self.assertEqual(state["positions_rad"], [0.1] * 6)
            self.assertEqual(state["velocities_rad_sec"], [0.0] * 6)
            self.assertEqual(state["mode"], "Running")
            self.assertEqual(state["safety_mode"], "Normal")
            self.assertFalse(state["teach_enabled"])
            self.assertEqual(backend.hardware_min, min_positions)
            self.assertEqual(backend.hardware_max, max_positions)
            self.assertEqual(backend.payload[0], 0.8)
            self.assertEqual(backend.tcp_offset, [0.0] * 6)
        finally:
            backend.close()
            server.close()
            worker.join(timeout=1.0)
        self.assertEqual(worker_errors, [])

    def test_joint_vectors_reject_nonfinite_or_wrong_length(self):
        self.assertEqual(_six([0] * 6), [0.0] * 6)
        with self.assertRaises(ValueError):
            _six([0] * 5)
        with self.assertRaises(ValueError):
            _six([0, 0, 0, 0, float("nan"), 0])


if __name__ == "__main__":
    unittest.main()
