#!/usr/bin/env python3
"""Expose the ROS SIL graph as a loopback simulated firmware endpoint."""

from __future__ import annotations

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hexapod_msgs.msg import MotionCommand, SilSafetyInput

from companion_sim_protocol import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    DEFAULT_TOKEN,
    SimulatedFirmware,
    SimulatedFirmwareServer,
)


class CompanionSimulationBridge(Node):
    """Maps the companion binary protocol to the high-level SIL inputs."""

    def __init__(self) -> None:
        super().__init__("companion_sim_bridge")
        host = self.declare_parameter("host", DEFAULT_HOST).value
        port = int(self.declare_parameter("port", DEFAULT_PORT).value)
        token = self.declare_parameter("token", DEFAULT_TOKEN).value
        motion_topic = self.declare_parameter(
            "motion_topic", "hexapod_controller_core/motion_command"
        ).value
        safety_topic = self.declare_parameter(
            "safety_topic", "hexapod_controller_core/sil_safety"
        ).value
        rate_hz = int(self.declare_parameter("publish_rate_hz", 20).value)
        if rate_hz <= 0:
            raise ValueError("publish_rate_hz must be positive")

        self._motion_publisher = self.create_publisher(MotionCommand, motion_topic, 10)
        self._safety_publisher = self.create_publisher(SilSafetyInput, safety_topic, 10)
        self._firmware = SimulatedFirmware()
        self._server = SimulatedFirmwareServer(
            self._firmware, host=str(host), port=port, token=str(token)
        )
        self._server.start()
        self._timer = self.create_timer(1.0 / rate_hz, self._publish_sil_inputs)
        self._publish_sil_inputs()
        self.get_logger().info(
            "Companion simulation endpoint ready at "
            f"tcp://{host}:{self._server.port}?token={token}"
        )

    def _publish_sil_inputs(self) -> None:
        motion = self._firmware.motion_command()
        motion_message = MotionCommand()
        motion_message.header.stamp = self.get_clock().now().to_msg()
        motion_message.sequence = motion.sequence
        motion_message.valid_for.sec = motion.valid_for_ms // 1000
        motion_message.valid_for.nanosec = (motion.valid_for_ms % 1000) * 1_000_000
        motion_message.gait = motion.gait
        motion_message.body_height_m = motion.body_height_m
        motion_message.stride_length_m = motion.stride_length_m
        motion_message.step_height_m = motion.step_height_m
        motion_message.duty_factor = motion.duty_factor
        motion_message.speed_scale = motion.speed_scale
        motion_message.normalized_vx = motion.normalized_vx
        motion_message.normalized_vy = motion.normalized_vy
        motion_message.normalized_wz = motion.normalized_wz
        motion_message.body_translation_m.x = motion.body_x_m
        motion_message.body_translation_m.y = motion.body_y_m
        motion_message.body_translation_m.z = motion.body_z_m
        motion_message.body_rpy_rad.x = motion.roll_rad
        motion_message.body_rpy_rad.y = motion.pitch_rad
        motion_message.body_rpy_rad.z = motion.yaw_rad
        self._motion_publisher.publish(motion_message)

        state = self._firmware.safety_state()
        safety_message = SilSafetyInput()
        safety_message.header.stamp = motion_message.header.stamp
        safety_message.rc_seen = True
        safety_message.rc_kill = state.host_estop
        safety_message.rc_armed = state.armed
        safety_message.rc_autonomy_enabled = True
        safety_message.host_estop = state.host_estop
        safety_message.battery_valid = True
        safety_message.battery_millivolts = 12000
        safety_message.watchdog_fault = False
        safety_message.dxl_hard_fault = False
        safety_message.foot_contact_enabled = False
        safety_message.terrain_leveling_enabled = False
        self._safety_publisher.publish(safety_message)

    def destroy_node(self) -> bool:
        self._server.stop()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node: CompanionSimulationBridge | None = None
    try:
        node = CompanionSimulationBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Launch-driven SIGINT/SIGTERM shutdown is a normal exit, not an error.
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())