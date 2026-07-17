"""Loopback coverage for the ROS simulated-firmware protocol core."""

from __future__ import annotations

import pathlib
import sys
import time
import unittest

from hexapod_protocol import api, telemetry as tlm


SCRIPT_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from companion_sim_protocol import (  # noqa: E402
    DEFAULT_TOKEN,
    SimulatedFirmware,
    SimulatedFirmwareServer,
)
from transport.tcp_proxy import connect_tcp_proxy  # noqa: E402
from transport.protocol_client import ProtocolClient  # noqa: E402


class CompanionSimulationProtocolTest(unittest.TestCase):
    def setUp(self) -> None:
        self.published = []
        self.firmware = SimulatedFirmware(self.published.append)
        self.server = SimulatedFirmwareServer(self.firmware, port=0)
        self.server.start()
        endpoint = f"tcp://127.0.0.1:{self.server.port}?token={DEFAULT_TOKEN}"
        self.client = ProtocolClient(connect_tcp_proxy(endpoint), response_timeout=0.5)
        self.client.start()

    def tearDown(self) -> None:
        self.client.stop()
        self.server.stop()

    def test_companion_protocol_maps_motion_and_disables_hardware_features(self) -> None:
        hello = self.client.hello()
        self.assertIsNotNone(hello)
        assert hello is not None
        self.assertEqual(hello.device_name, "HexNav ROS SIM")

        capabilities = self.client.get_capabilities()
        self.assertIsNotNone(capabilities)
        assert capabilities is not None
        self.assertEqual(capabilities.feature_bits, 0)

        features = self.client.feature_get()
        self.assertIsNotNone(features)
        assert features is not None
        self.assertEqual(len(features.features), api.FEATURE_COUNT)
        self.assertTrue(all(not feature.available for feature in features.features))

        rejected = self.client.feature_set(api.FEATURE_FOOT_CONTACT, True)
        self.assertIsNotNone(rejected)
        assert rejected is not None
        self.assertTrue(rejected.rejected)
        self.assertEqual(rejected.reason, api.FEATURE_REASON_NOT_IMPLEMENTED)

        self.assertTrue(self.client.set_gait(api.GAIT_RIPPLE).ok)
        self.assertTrue(self.client.set_gait_params(50, 75, 45, 200, 180).ok)
        self.assertTrue(self.client.set_body_twist(0.75, -0.5, 0.25).ok)
        self.assertTrue(self.client.set_body_pose(20, -10, 5, 12, -8, 4).ok)

        motion = self.firmware.motion_command()
        self.assertEqual(motion.gait, api.GAIT_RIPPLE)
        self.assertAlmostEqual(motion.body_height_m, 0.050)
        self.assertAlmostEqual(motion.stride_length_m, 0.075)
        self.assertAlmostEqual(motion.step_height_m, 0.045)
        self.assertAlmostEqual(motion.normalized_vx, 0.75)
        self.assertAlmostEqual(motion.normalized_vy, -0.5)
        self.assertAlmostEqual(motion.normalized_wz, 0.25)
        self.assertAlmostEqual(motion.body_x_m, 0.020)
        self.assertAlmostEqual(motion.body_y_m, -0.010)
        self.assertAlmostEqual(motion.roll_rad, 12 * 3.141592653589793 / 180)
        self.assertGreaterEqual(len(self.published), 4)

    def test_control_state_telemetry_reflects_simulated_arming(self) -> None:
        self.assertFalse(self.firmware.motion_allowed())

        armed = self.client.set_arming(True)
        self.assertIsNotNone(armed)
        assert armed is not None
        self.assertEqual(armed.state, int(tlm.SafetyState.RC_MANUAL))
        self.assertTrue(self.firmware.motion_allowed())

        received = []
        self.client.on_telemetry(lambda stream, record, _header: received.append((stream, record)))
        subscribed = self.client.subscribe(int(tlm.StreamId.CONTROL_STATE), 20)
        self.assertIsNotNone(subscribed)
        assert subscribed is not None
        self.assertTrue(subscribed.ok)

        deadline = time.monotonic() + 1.0
        while not received and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(received)
        stream, record = received[-1]
        self.assertEqual(stream, int(tlm.StreamId.CONTROL_STATE))
        self.assertTrue(record.motion_gate)

        disarm = self.client.set_arming(False)
        self.assertIsNotNone(disarm)
        assert disarm is not None
        self.assertEqual(disarm.state, int(tlm.SafetyState.DISARMED))
        self.assertFalse(self.firmware.motion_allowed())


if __name__ == "__main__":
    unittest.main()