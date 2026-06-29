"""UI-independent hexapod kinematic pose model.

Computes per-leg joint chains (hip -> femur base -> knee -> foot) in the body
frame from URDF-zero-relative joint angles, mirroring the firmware forward
kinematics (``gait::LegIk`` / ``gait::BodyKinematics``).

The model is fed by:

* the firmware ``joint_state`` telemetry stream (eax.1) -- preferred, or
* ``servo_status`` ticks + the config servo map (eax.4 host fallback).

It contains no Qt/pyqtgraph dependency so it can be unit-tested headless and
reused for live, replay, and URDF feeds.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from hexapod_protocol import config as cfg
from hexapod_protocol import telemetry as tlm

# Reference stance/coxa geometry, used as a fallback when a config omits the
# persisted BodyGeometry block (firmware lmt.11). Mirror gait::kCoxaLiftMm and
# gait::kHomeRadiusMm / kHomeFootZMm. All-zero joint angles -> the home foot in
# the coxa frame.
COXA_LIFT_MM = 21.0
HOME_RADIUS_MM = 127.0
HOME_FOOT_Z_MM = -44.55

_HALF_PI = math.pi / 2.0


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


@dataclass
class LegPose:
    """One leg's solved chain in the body frame (mm) plus its joint angles."""

    leg: int
    coxa_rad: float = 0.0
    femur_rad: float = 0.0
    tibia_rad: float = 0.0
    hip: Point3 = field(default_factory=lambda: Point3(0.0, 0.0, 0.0))
    femur_base: Point3 = field(default_factory=lambda: Point3(0.0, 0.0, 0.0))
    knee: Point3 = field(default_factory=lambda: Point3(0.0, 0.0, 0.0))
    foot: Point3 = field(default_factory=lambda: Point3(0.0, 0.0, 0.0))

    @property
    def points(self) -> list[Point3]:
        """The drawable chain, hip -> femur base -> knee -> foot."""
        return [self.hip, self.femur_base, self.knee, self.foot]


class _LegFk:
    """Forward kinematics for a single leg in its coxa frame.

    Mirrors ``gait::LegIk``: ``l1`` is the radial coxa->femur offset, ``l2`` the
    femur link, ``l3`` the tibia (foot tip) link. The rest angles make the
    documented home foot map to all-zero joint angles, so URDF-zero-relative
    angles (as carried by ``joint_state``) feed straight in.
    """

    def __init__(
        self,
        l1_mm: float,
        l2_mm: float,
        l3_mm: float,
        home_radius_mm: float = HOME_RADIUS_MM,
        home_foot_z_mm: float = HOME_FOOT_Z_MM,
    ) -> None:
        self.l1 = l1_mm
        self.l2 = l2_mm
        self.l3 = l3_mm
        self._femur_rest, self._tibia_rest = self._solve_raw_planar(
            home_radius_mm, home_foot_z_mm
        )

    def _solve_raw_planar(self, horiz: float, dz: float) -> tuple[float, float]:
        """Raw femur/tibia planar angles for a foot at (horiz radial, dz) mm."""
        planar_r = horiz - self.l1
        d = math.hypot(planar_r, dz)
        cos_k = (d * d - self.l2 * self.l2 - self.l3 * self.l3) / (
            2.0 * self.l2 * self.l3
        )
        cos_k = max(-1.0, min(1.0, cos_k))
        beta = math.acos(cos_k)
        a = math.atan2(dz, planar_r)
        b = math.atan2(self.l3 * math.sin(beta), self.l2 + self.l3 * math.cos(beta))
        return a - b, beta

    def chain(
        self, coxa: float, femur: float, tibia: float
    ) -> tuple[Point3, Point3, Point3, Point3]:
        """Coxa-frame chain points for URDF-zero-relative joint angles (rad)."""
        alpha = femur + self._femur_rest  # raw femur
        beta = tibia + self._tibia_rest  # raw tibia
        c = math.cos(coxa)
        s = math.sin(coxa)

        # Radial distance / vertical at each joint along the planar arm.
        r_hip = 0.0
        r_femur = self.l1
        r_knee = self.l1 + self.l2 * math.cos(alpha)
        z_knee = self.l2 * math.sin(alpha)
        r_foot = r_knee + self.l3 * math.cos(alpha + beta)
        z_foot = z_knee + self.l3 * math.sin(alpha + beta)

        return (
            Point3(r_hip * c, r_hip * s, 0.0),
            Point3(r_femur * c, r_femur * s, 0.0),
            Point3(r_knee * c, r_knee * s, z_knee),
            Point3(r_foot * c, r_foot * s, z_foot),
        )


@dataclass
class _LegXform:
    """Coxa frame -> body frame placement (inverse of footBodyToCoxa)."""

    hip_x_mm: float
    hip_y_mm: float
    z_off_mm: float
    cos_a: float
    sin_a: float

    def to_body(self, p: Point3) -> Point3:
        # Inverse of: cx = cos_a*dx - sin_a*dy ; cy = sin_a*dx + cos_a*dy.
        dx = self.cos_a * p.x + self.sin_a * p.y
        dy = -self.sin_a * p.x + self.cos_a * p.y
        return Point3(self.hip_x_mm + dx, self.hip_y_mm + dy, self.z_off_mm + p.z)


class HexapodPoseModel:
    """Mutable per-leg pose state driven by joint telemetry.

    Build from a :class:`RobotConfig`; feed live or replayed joint angles and
    read back :class:`LegPose` chains in the body frame for drawing.
    """

    def __init__(self, config: cfg.RobotConfig) -> None:
        self._cfg = config
        # Stance/coxa geometry comes from the persisted config (firmware lmt.11).
        # A zero home radius means the config predates / omits the geometry
        # block, so fall back to the documented reference nominals.
        geom = config.geometry
        if geom.home_radius_cmm:
            home_radius_mm = geom.home_radius_cmm / 100.0
            home_foot_z_mm = geom.home_foot_z_cmm / 100.0
            coxa_lift_mm = geom.coxa_lift_cmm / 100.0
        else:
            home_radius_mm = HOME_RADIUS_MM
            home_foot_z_mm = HOME_FOOT_Z_MM
            coxa_lift_mm = COXA_LIFT_MM
        self._fk = _LegFk(
            config.links.coxa_cmm / 100.0,
            config.links.femur_cmm / 100.0,
            config.links.tibia_cmm / 100.0,
            home_radius_mm,
            home_foot_z_mm,
        )
        self._xforms: list[_LegXform] = []
        for leg in config.legs:
            yaw = leg.mount_yaw_cdeg * (math.pi / 180.0) / 100.0
            a = -(yaw + _HALF_PI)
            self._xforms.append(
                _LegXform(
                    hip_x_mm=leg.mount_x_dmm / 10.0,
                    hip_y_mm=leg.mount_y_dmm / 10.0,
                    z_off_mm=leg.mount_z_dmm / 10.0 + coxa_lift_mm,
                    cos_a=math.cos(a),
                    sin_a=math.sin(a),
                )
            )
        self._angles: list[list[float]] = [[0.0, 0.0, 0.0] for _ in config.legs]
        self._legs: list[LegPose] = [LegPose(leg=i) for i in range(len(config.legs))]
        self._recompute_all()

    @property
    def num_legs(self) -> int:
        return len(self._cfg.legs)

    def legs(self) -> list[LegPose]:
        """Snapshot list of solved leg poses (one per leg)."""
        return list(self._legs)

    def leg(self, index: int) -> LegPose:
        return self._legs[index]

    # --- feeds ------------------------------------------------------------

    def set_joint_angle(self, leg: int, joint: int, angle_rad: float) -> None:
        if 0 <= leg < self.num_legs and 0 <= joint < cfg.JOINTS_PER_LEG:
            self._angles[leg][joint] = angle_rad
            self._recompute(leg)

    def update_from_joint_state(self, record: tlm.JointStateTelemetry) -> None:
        """Apply a ``joint_state`` telemetry record (angles in centidegrees)."""
        touched: set[int] = set()
        for j in record.joints:
            if 0 <= j.leg < self.num_legs and 0 <= j.joint < cfg.JOINTS_PER_LEG:
                self._angles[j.leg][j.joint] = math.radians(j.angle_centideg / 100.0)
                touched.add(j.leg)
        for leg in touched:
            self._recompute(leg)

    def update_from_servo_status(self, record: tlm.ServoStatusTelemetry) -> None:
        """Apply ``servo_status`` ticks via the config servo-map fallback."""
        joints = cfg.servo_status_to_joint_angles(self._cfg, record)
        self.update_from_joint_state(tlm.JointStateTelemetry(joints=joints))

    # --- internals --------------------------------------------------------

    def _recompute(self, leg: int) -> None:
        coxa, femur, tibia = self._angles[leg]
        hip, fbase, knee, foot = self._fk.chain(coxa, femur, tibia)
        xf = self._xforms[leg]
        self._legs[leg] = LegPose(
            leg=leg,
            coxa_rad=coxa,
            femur_rad=femur,
            tibia_rad=tibia,
            hip=xf.to_body(hip),
            femur_base=xf.to_body(fbase),
            knee=xf.to_body(knee),
            foot=xf.to_body(foot),
        )

    def _recompute_all(self) -> None:
        for leg in range(self.num_legs):
            self._recompute(leg)
