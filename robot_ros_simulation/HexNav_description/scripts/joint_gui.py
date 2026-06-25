#!/usr/bin/env python3
"""HexNav joint slider GUI.

A small Tkinter control panel with one slider per actuated joint. Sliders are
expressed in real Dynamixel MX-28 servo angles (degrees, mechanically centered
at 180 deg) and each joint has an "inv" checkbox to invert its direction when
the servo is mounted mirrored. Slider values are converted to radians and
published as a std_msgs/Float64MultiArray to the ros2_control
forward_command_controller, which drives the mock hardware and is reflected in
RViz through /joint_states.

Usage:
    ros2 run HexNav_description joint_gui.py
    ros2 run HexNav_description joint_gui.py --ros-args -p command_topic:=/position_controller/commands
"""

import math
import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Joints in the exact order expected by the controller (see
# config/ros2_controllers.yaml). Grouped by leg: coxa, femur, tibia.
LEGS = [
    ("Leg 1", ["leg_1_coxa_joint", "leg_1_femur_joint", "leg_1_tibia_joint"]),
    ("Leg 2", ["leg_2_coxa_joint", "leg_2_femur_joint", "leg_2_tibia_joint"]),
    ("Leg 3", ["leg_3_coxa_joint", "leg_3_femur_joint", "leg_3_tibia_joint"]),
    ("Leg 4", ["leg_4_coxa_joint", "leg_4_femur_joint", "leg_4_tibia_joint"]),
    ("Leg 5", ["leg_5_coxa_joint", "leg_5_femur_joint", "leg_5_tibia_joint"]),
    ("Leg 6", ["leg_6_coxa_joint", "leg_6_femur_joint", "leg_6_tibia_joint"]),
]
SEGMENT_LABELS = ["coxa", "femur", "tibia"]

# Ordered list of all joints, matching the controller command vector.
JOINT_ORDER = [name for _, joints in LEGS for name in joints]

# Dynamixel MX-28 servo convention. The MX-28 uses a 12-bit contactless
# absolute encoder (4096 pulses/rev, 0.088 deg/pulse). In Joint Mode the Goal
# Position spans the full 0-360 deg range; the robot's home/start pose keeps
# every servo at 180 deg (the mechanical center of travel), which maps to the
# URDF joint's neutral (0 rad) pose.
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 360.0
SERVO_CENTER_DEG = 180.0
SERVO_RESOLUTION_DEG = 0.088


def servo_deg_to_joint_rad(servo_deg, inverted):
    """Convert a real MX-28 servo angle (deg, centered at 180) to a joint
    command in radians, honoring the per-servo inversion flag."""
    delta = math.radians(servo_deg - SERVO_CENTER_DEG)
    return -delta if inverted else delta


def joint_rad_to_servo_deg(joint_rad, inverted):
    """Inverse of servo_deg_to_joint_rad: map a joint angle (rad) back to the
    real MX-28 servo angle in degrees."""
    delta = -joint_rad if inverted else joint_rad
    return SERVO_CENTER_DEG + math.degrees(delta)


class JointGui(Node):
    def __init__(self):
        super().__init__("hexnav_joint_gui")
        self.declare_parameter("command_topic", "/position_controller/commands")
        topic = self.get_parameter("command_topic").get_parameter_value().string_value

        self._pub = self.create_publisher(Float64MultiArray, topic, 10)
        # Real servo angles in degrees (MX-28), centered at 180 deg, plus a
        # per-servo inversion flag.
        self._servo_deg = {name: SERVO_CENTER_DEG for name in JOINT_ORDER}
        self._inverted = {name: False for name in JOINT_ORDER}
        self._lock = threading.Lock()

        # Publish at a steady rate so commands are latched even without GUI motion.
        self.create_timer(0.05, self._publish)
        self.get_logger().info(f"Publishing joint commands on: {topic}")

    def set_servo_deg(self, name, value):
        with self._lock:
            self._servo_deg[name] = float(value)

    def set_inverted(self, name, inverted):
        with self._lock:
            self._inverted[name] = bool(inverted)

    def reset(self):
        with self._lock:
            for name in self._servo_deg:
                self._servo_deg[name] = SERVO_CENTER_DEG

    def snapshot(self):
        with self._lock:
            return [
                servo_deg_to_joint_rad(self._servo_deg[name], self._inverted[name])
                for name in JOINT_ORDER
            ]

    def _publish(self):
        msg = Float64MultiArray()
        msg.data = self.snapshot()
        self._pub.publish(msg)


def build_ui(node):
    root = tk.Tk()
    root.title("HexNav Joint Control — MX-28 servos (deg)")

    sliders = []

    container = tk.Frame(root, padx=8, pady=8)
    container.pack(fill="both", expand=True)

    for col, (leg_label, joints) in enumerate(LEGS):
        leg_frame = tk.LabelFrame(container, text=leg_label, padx=6, pady=6)
        leg_frame.grid(row=0, column=col, padx=4, pady=4, sticky="n")

        for seg_label, joint in zip(SEGMENT_LABELS, joints):
            row = tk.Frame(leg_frame)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=seg_label, width=5, anchor="w").pack(side="left")

            invert_var = tk.BooleanVar(value=False)

            def make_invert_cb(joint_name=joint, variable=invert_var):
                def _cb():
                    node.set_inverted(joint_name, variable.get())

                return _cb

            tk.Checkbutton(
                row,
                text="inv",
                variable=invert_var,
                command=make_invert_cb(),
            ).pack(side="left")

            var = tk.DoubleVar(value=SERVO_CENTER_DEG)

            def make_cb(joint_name=joint, variable=var):
                def _cb(_value):
                    node.set_servo_deg(joint_name, variable.get())

                return _cb

            scale = tk.Scale(
                row,
                from_=SERVO_MIN_DEG,
                to=SERVO_MAX_DEG,
                resolution=SERVO_RESOLUTION_DEG,
                orient="horizontal",
                length=150,
                variable=var,
                command=make_cb(),
                showvalue=True,
            )
            scale.pack(side="left")
            sliders.append((var, scale))

    def reset_all():
        node.reset()
        for var, _scale in sliders:
            var.set(SERVO_CENTER_DEG)

    button_bar = tk.Frame(root, padx=8, pady=6)
    button_bar.pack(fill="x")
    tk.Button(button_bar, text="Center (180°)", command=reset_all).pack(side="left")
    tk.Label(
        button_bar,
        text="MX-28 Joint Mode 0–360°, start 180°  •  'inv' flips direction",
    ).pack(side="right")

    return root


def main():
    rclpy.init()
    node = JointGui()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    root = build_ui(node)

    def on_close():
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
