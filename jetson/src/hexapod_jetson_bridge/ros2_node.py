"""Optional ROS 2 wrapper for forwarding high-level motion to JetsonBridge."""

from __future__ import annotations

from importlib import import_module
from typing import Protocol, Sequence

from .motion_command_adapter import MotionCommandAdapter, MotionCommandInput
from .serial_client import JetsonBridge


class DurationLike(Protocol):
    sec: int
    nanosec: int


class Vector3Like(Protocol):
    x: float
    y: float
    z: float


class MotionCommandLike(Protocol):
    gait: int
    body_height_m: float
    stride_length_m: float
    step_height_m: float
    duty_factor: float
    speed_scale: float
    normalized_vx: float
    normalized_vy: float
    normalized_wz: float
    body_translation_m: Vector3Like
    body_rpy_rad: Vector3Like
    valid_for: DurationLike


def motion_command_from_ros(message: MotionCommandLike) -> MotionCommandInput:
    """Copy a generated ROS message into the dependency-free adapter input."""
    return MotionCommandInput(
        gait=message.gait,
        body_height_m=message.body_height_m,
        stride_length_m=message.stride_length_m,
        step_height_m=message.step_height_m,
        duty_factor=message.duty_factor,
        speed_scale=message.speed_scale,
        normalized_vx=message.normalized_vx,
        normalized_vy=message.normalized_vy,
        normalized_wz=message.normalized_wz,
        body_translation_x_m=message.body_translation_m.x,
        body_translation_y_m=message.body_translation_m.y,
        body_translation_z_m=message.body_translation_m.z,
        body_roll_rad=message.body_rpy_rad.x,
        body_pitch_rad=message.body_rpy_rad.y,
        body_yaw_rad=message.body_rpy_rad.z,
        valid_for_sec=message.valid_for.sec,
        valid_for_nanosec=message.valid_for.nanosec,
    )


def run_ros_node(argv: Sequence[str] | None = None) -> None:
    """Run the rclpy node only when ROS 2 is available at runtime."""
    rclpy = import_module("rclpy")
    node_module = import_module("rclpy.node")
    messages_module = import_module("hexapod_msgs.msg")
    motion_message_type = getattr(messages_module, "MotionCommand")

    class JetsonRosBridgeNode(node_module.Node):
        def __init__(self) -> None:
            super().__init__("hexapod_jetson_bridge")
            self.declare_parameter("serial_port", "")
            self.declare_parameter("baud", 115200)
            self.declare_parameter("motion_topic", "motion_command")
            self.declare_parameter("ttl_check_period_ms", 20)

            serial_port = str(self.get_parameter("serial_port").value)
            if not serial_port:
                raise RuntimeError("serial_port must name the OpenRB-150 USB device")
            baud = int(self.get_parameter("baud").value)
            bridge = JetsonBridge.connect(serial_port, baud=baud)
            if bridge is None:
                raise RuntimeError(f"could not connect to OpenRB-150 at {serial_port}")

            self._bridge = bridge
            self._motion_adapter = MotionCommandAdapter(bridge)
            motion_topic = str(self.get_parameter("motion_topic").value)
            self._subscription = self.create_subscription(
                motion_message_type,
                motion_topic,
                self._on_motion_command,
                10,
            )
            period_ms = int(self.get_parameter("ttl_check_period_ms").value)
            if period_ms <= 0:
                raise RuntimeError("ttl_check_period_ms must be positive")
            self._ttl_timer = self.create_timer(
                period_ms / 1000.0, self._on_ttl_timer
            )
            self.get_logger().info(
                f"forwarding {motion_topic} to {serial_port} at {baud} baud"
            )

        def _on_motion_command(self, message: MotionCommandLike) -> None:
            command = motion_command_from_ros(message)
            if not self._motion_adapter.accept(command):
                error = self._motion_adapter.last_error or "motion command rejected"
                self.get_logger().warning(error)

        def _on_ttl_timer(self) -> None:
            if self._motion_adapter.expire_if_needed():
                self.get_logger().warning("motion command expired; sent STOP_MOTION")

        def destroy_node(self) -> object:
            try:
                self._bridge.stop_motion()
            finally:
                self._bridge.close()
            return super().destroy_node()

    node = None
    rclpy.init(args=list(argv) if argv is not None else None)
    try:
        node = JetsonRosBridgeNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    """Console-script entry point for a Pixi/RoboStack ROS environment."""
    run_ros_node()


if __name__ == "__main__":
    main()