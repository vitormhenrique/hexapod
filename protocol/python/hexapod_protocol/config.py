"""Persistent robot config decoder + servo-map / tick<->angle helpers (host side).

Mirrors ``firmware/openrb150/src/config/config_schema.{h,cpp}`` (the serialized
``RobotConfig`` payload that CFG_GET_BLOCK windows transfer) and
``firmware/openrb150/src/dxl/servo_map.{h,cpp}`` (the URDF-zero-relative
tick<->angle conversion). UI-independent so the CLI, tests, and the PySide6 app
all share one decoder.

The serialized config payload is larger than one protocol frame, so the host
reassembles it from CFG_GET_BLOCK windows (see :class:`ConfigBlockAssembler`)
before calling :func:`decode_robot_config`. When the firmware ``joint_state``
telemetry stream (eax.1) is unavailable, :func:`servo_status_to_joint_angles`
reproduces the same mapped angles on the host from raw ``servo_status`` ticks
using this config's servo map.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass, field

# --------------------------------------------------------------------------- #
# Fixed dimensions / constants (mirror config_schema.h).
# --------------------------------------------------------------------------- #
NUM_LEGS = 6
JOINTS_PER_LEG = 3
NUM_SERVOS = NUM_LEGS * JOINTS_PER_LEG  # 18
NUM_FOOT_SENSORS = 6
NUM_RC_ANALOG_INPUTS = 8
ROBOT_NAME_LEN = 16  # incl. NUL terminator
# v3: default joint trims are zero -- servo center (2048/180 deg) is the
# mechanical home pose per the URDF all-zero model. The bump invalidates v2
# payloads that carried the old hidden +171/-512 posture trims.
# v4 adds logical RC calibration.
# v5 adds central body-command limits.
# v6 installs the Mark III motion profile.
# v7 restores the assembled robot's verified sequential DXL IDs 1..18 while
# retaining Mark III geometry, angles, side inversions, and gait timing.
# v8 restores HexNav CAD dimensions and zero-centered servo calibration.
SCHEMA_VERSION = 9
LEGACY_SCHEMA_VERSION_V8 = 8
LEGACY_SCHEMA_VERSION_V7 = 7
LEGACY_SCHEMA_VERSION_V6 = 6
LEGACY_SCHEMA_VERSION_V5 = 5
LEGACY_SCHEMA_VERSION_V4 = 4
LEGACY_SCHEMA_VERSION_V3 = 3

SERVO_CENTER_TICK = 2048
SERVO_MAX_TICK = 4095
DEFAULT_COXA_TRIM_TICKS = 0
DEFAULT_FEMUR_TRIM_TICKS = 0
DEFAULT_TIBIA_TRIM_TICKS = 0
TICKS_PER_REV = 4096.0
TICKS_PER_DEG = TICKS_PER_REV / 360.0  # ~11.3778
RAD_TO_DEG = 180.0 / math.pi
DEG_TO_RAD = math.pi / 180.0

# Largest config window per CFG_GET_BLOCK/CFG_SET_BLOCK frame (config_api.h).
CFG_BLOCK_MAX = 192

# Serialized payload size (bytes), mirroring kConfigPayloadSize.
CONFIG_PAYLOAD_SIZE = (
    2  # schema_version
    + ROBOT_NAME_LEN  # robot_name
    + 3 * 2  # links (3 x uint16)
    + (2 + 2 + 2)  # body geometry (u16 radius, i16 z, u16 lift)
    + NUM_LEGS * (4 * 2)  # legs (4 x int16 each)
    + NUM_SERVOS * (1 + 1 + 1 + 1 + 2 + 2 + 2)  # servos (10 bytes each)
    + (2 + 2 + 2 + 1 + 1 + 1)  # gait (9 bytes)
    + NUM_RC_ANALOG_INPUTS * (1 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 2)
    # logical RC calibration (15 bytes each)
    + 14 * 2  # body-command limits (14 x uint16)
    + NUM_FOOT_SENSORS * (4 + 2 + 2 + 2 + 1)  # feet (11 bytes each)
    + 4  # feature_defaults
)  # == 485

LEGACY_CONFIG_PAYLOAD_SIZE_V3 = (
    2
    + ROBOT_NAME_LEN
    + 3 * 2
    + (2 + 2 + 2)
    + NUM_LEGS * (4 * 2)
    + NUM_SERVOS * (1 + 1 + 1 + 1 + 2 + 2 + 2)
    + (2 + 2 + 2 + 1 + 1 + 1)
    + NUM_FOOT_SENSORS * (4 + 2 + 2 + 2 + 1)
    + 4
)

LEGACY_CONFIG_PAYLOAD_SIZE_V4 = (
    LEGACY_CONFIG_PAYLOAD_SIZE_V3
    + NUM_RC_ANALOG_INPUTS * (1 + 1 + 2 + 2 + 2 + 1 + 1 + 1 + 2 + 2)
)

RC_CENTERED_ANALOG = 0
RC_UNIPOLAR_ANALOG = 1
RC_RELATIVE_ENCODER = 2
RC_FILTER_TAU_MAX_MS = 1000
RC_SWITCH_DEBOUNCE_MAX_MS = 2000
BODY_COMMAND_MAX_SCALE_MILLI = 1000
BODY_COMMAND_MAX_ACCEL_MILLI_PER_S = 5000
BODY_HEIGHT_RATE_MAX_MM_PER_S = 200
BODY_POSE_TRANSLATION_RATE_MAX_MM_PER_S = 200
BODY_POSE_ROTATION_RATE_MAX_MILLIRAD_PER_S = 2000

# Gait ids (mirror config::GaitId).
GAIT_NAMES = {0: "stand", 1: "sit", 2: "tripod", 3: "ripple", 4: "wave", 5: "crawl"}

# Joint roles (mirror config::JointRole).
JOINT_ROLE_NAMES = {0: "coxa", 1: "femur", 2: "tibia"}

# Feature default bits (mirror config::FeatureBit).
FEAT_FOOT_CONTACT = 1 << 0
FEAT_TERRAIN_LEVELING = 1 << 1
FEAT_SENSOR_POLLING = 1 << 2
FEAT_PASSIVE_POSE_STREAM = 1 << 3
FEAT_JETSON_CONTROL = 1 << 4


# --------------------------------------------------------------------------- #
# Dataclasses (mirror the config_schema.h sub-structures).
# --------------------------------------------------------------------------- #
@dataclass
class LinkLengths:
    coxa_cmm: int = 0  # 0.01 mm
    femur_cmm: int = 0  # 0.01 mm
    tibia_cmm: int = 0  # 0.01 mm


@dataclass
class BodyGeometry:
    home_radius_cmm: int = 0  # 0.01 mm, neutral foot radial distance
    home_foot_z_cmm: int = 0  # 0.01 mm, neutral foot height (negative = down)
    coxa_lift_cmm: int = 0  # 0.01 mm, coxa axis lift above the body mount


@dataclass
class LegGeometry:
    mount_x_dmm: int = 0  # 0.1 mm
    mount_y_dmm: int = 0  # 0.1 mm
    mount_z_dmm: int = 0  # 0.1 mm
    mount_yaw_cdeg: int = 0  # 0.01 deg


@dataclass
class ServoConfig:
    id: int = 0
    leg: int = 0
    joint: int = 0
    sign: int = 1  # +1 or -1
    trim_ticks: int = 0  # signed offset added to center
    min_tick: int = 0
    max_tick: int = SERVO_MAX_TICK

    @property
    def joint_name(self) -> str:
        return JOINT_ROLE_NAMES.get(self.joint, f"0x{self.joint:02X}")


@dataclass
class GaitDefaults:
    body_height_mm: int = 0
    stride_len_mm: int = 0
    step_height_mm: int = 0
    duty_x255: int = 0
    speed_x255: int = 0
    gait: int = 0

    @property
    def gait_name(self) -> str:
        return GAIT_NAMES.get(self.gait, f"0x{self.gait:02X}")


@dataclass
class RcChannelCalibration:
    source: int = 0
    type: int = RC_CENTERED_ANALOG
    min_raw: int = 0
    center_raw: int = 0
    max_raw: int = 0
    reversed: int = 0
    deadband_x255: int = 0
    expo_x255: int = 0
    filter_tau_ms: int = 0
    switch_debounce_ms: int = 0


@dataclass
class RcInputCalibration:
    channels: list[RcChannelCalibration] = field(default_factory=list)


@dataclass
class BodyCommandLimits:
    max_forward_milli: int = 1000
    max_reverse_milli: int = 1000
    max_lateral_milli: int = 1000
    max_yaw_milli: int = 1000
    forward_accel_milli_per_s: int = 1200
    forward_decel_milli_per_s: int = 1800
    lateral_accel_milli_per_s: int = 1000
    lateral_decel_milli_per_s: int = 1500
    yaw_accel_milli_per_s: int = 1500
    yaw_decel_milli_per_s: int = 2000
    height_rise_mm_per_s: int = 20
    height_lower_mm_per_s: int = 30
    pose_translation_rate_mm_per_s: int = 60
    pose_rotation_rate_millirad_per_s: int = 500


@dataclass
class FootSensorCal:
    pressure_baseline: int = 0
    near_thresh: int = 0
    touch_thresh: int = 0
    load_thresh: int = 0
    enabled: int = 0


@dataclass
class RobotConfig:
    schema_version: int = SCHEMA_VERSION
    robot_name: str = ""
    links: LinkLengths = field(default_factory=LinkLengths)
    geometry: BodyGeometry = field(default_factory=BodyGeometry)
    legs: list[LegGeometry] = field(default_factory=list)
    servos: list[ServoConfig] = field(default_factory=list)
    gait: GaitDefaults = field(default_factory=GaitDefaults)
    rc_input: RcInputCalibration = field(
        default_factory=lambda: default_rc_input_calibration()
    )
    body_command: BodyCommandLimits = field(default_factory=BodyCommandLimits)
    feet: list[FootSensorCal] = field(default_factory=list)
    feature_defaults: int = 0


@dataclass
class ConfigSummary:
    """Decoded CFG_GET_SUMMARY response (config_api.cpp kGetSummary)."""

    schema_version: int
    payload_size: int
    block_max: int
    persistent: bool
    staged_valid: bool
    feature_defaults: int
    robot_name: str


# --------------------------------------------------------------------------- #
# (De)serialization (mirror serializeRobotConfig / deserializeRobotConfig).
# --------------------------------------------------------------------------- #
class ConfigDecodeError(ValueError):
    """Raised when a config payload is the wrong length or schema version."""


def _trim_name(raw: bytes) -> str:
    return raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")


def decode_robot_config(payload: bytes) -> RobotConfig:
    """Decode a serialized ``RobotConfig`` payload (little-endian, version-tagged).

    Raises :class:`ConfigDecodeError` if the length or schema version does not
    match, mirroring ``deserializeRobotConfig`` returning false.
    """
    if len(payload) not in (
        CONFIG_PAYLOAD_SIZE,
        LEGACY_CONFIG_PAYLOAD_SIZE_V4,
        LEGACY_CONFIG_PAYLOAD_SIZE_V3,
    ):
        raise ConfigDecodeError(
            "config payload is "
            f"{len(payload)} bytes, expected {CONFIG_PAYLOAD_SIZE}"
        )

    o = 0
    schema_version = struct.unpack_from("<H", payload, o)[0]
    o += 2
    legacy_v3 = (
        schema_version == LEGACY_SCHEMA_VERSION_V3
        and len(payload) == LEGACY_CONFIG_PAYLOAD_SIZE_V3
    )
    legacy_v4 = (
        schema_version == LEGACY_SCHEMA_VERSION_V4
        and len(payload) == LEGACY_CONFIG_PAYLOAD_SIZE_V4
    )
    legacy_v5 = (
        schema_version == LEGACY_SCHEMA_VERSION_V5
        and len(payload) == CONFIG_PAYLOAD_SIZE
    )
    legacy_v6 = (
        schema_version == LEGACY_SCHEMA_VERSION_V6
        and len(payload) == CONFIG_PAYLOAD_SIZE
    )
    legacy_v7 = (
        schema_version == LEGACY_SCHEMA_VERSION_V7
        and len(payload) == CONFIG_PAYLOAD_SIZE
    )
    legacy_v8 = (
        schema_version == LEGACY_SCHEMA_VERSION_V8
        and len(payload) == CONFIG_PAYLOAD_SIZE
    )
    if not legacy_v3 and not legacy_v4 and not legacy_v5 and not legacy_v6 and not legacy_v7 and not legacy_v8 and (
        schema_version != SCHEMA_VERSION or len(payload) != CONFIG_PAYLOAD_SIZE
    ):
        raise ConfigDecodeError(
            f"unsupported config schema version {schema_version} (expected {SCHEMA_VERSION})"
        )

    robot_name = _trim_name(payload[o : o + ROBOT_NAME_LEN])
    o += ROBOT_NAME_LEN

    links = LinkLengths(*struct.unpack_from("<HHH", payload, o))
    o += 6

    geometry = BodyGeometry(*struct.unpack_from("<HhH", payload, o))
    o += 6

    legs: list[LegGeometry] = []
    for _ in range(NUM_LEGS):
        legs.append(LegGeometry(*struct.unpack_from("<hhhh", payload, o)))
        o += 8

    servos: list[ServoConfig] = []
    for _ in range(NUM_SERVOS):
        sid, leg, joint, sign, trim, mn, mx = struct.unpack_from("<BBBbhHH", payload, o)
        servos.append(ServoConfig(sid, leg, joint, sign, trim, mn, mx))
        o += 10

    body_h, stride, step_h, duty, speed, gait = struct.unpack_from(
        "<HHHBBB", payload, o
    )
    o += 9
    gait_defaults = GaitDefaults(body_h, stride, step_h, duty, speed, gait)

    if legacy_v3:
        rc_input = default_rc_input_calibration()
    else:
        channels: list[RcChannelCalibration] = []
        for _ in range(NUM_RC_ANALOG_INPUTS):
            source, channel_type, minimum, center, maximum, reverse, deadband, expo, tau, debounce = (
                struct.unpack_from("<BBhhhBBBHH", payload, o)
            )
            channels.append(
                RcChannelCalibration(
                    source,
                    channel_type,
                    minimum,
                    center,
                    maximum,
                    reverse,
                    deadband,
                    expo,
                    tau,
                    debounce,
                )
            )
            o += 15
        rc_input = RcInputCalibration(channels)

    if legacy_v3 or legacy_v4:
        body_command = BodyCommandLimits()
    else:
        body_command = BodyCommandLimits(*struct.unpack_from("<" + "H" * 14, payload, o))
        o += 28

    feet: list[FootSensorCal] = []
    for _ in range(NUM_FOOT_SENSORS):
        baseline, near, touch, load, enabled = struct.unpack_from("<iHHHB", payload, o)
        feet.append(FootSensorCal(baseline, near, touch, load, enabled))
        o += 11

    feature_defaults = struct.unpack_from("<I", payload, o)[0]
    o += 4

    config = RobotConfig(
        schema_version=(
            SCHEMA_VERSION
            if (legacy_v3 or legacy_v4 or legacy_v5 or legacy_v6 or legacy_v7
                or legacy_v8)
            else schema_version
        ),
        robot_name=robot_name,
        links=links,
        geometry=geometry,
        legs=legs,
        servos=servos,
        gait=gait_defaults,
        rc_input=rc_input,
        body_command=body_command,
        feet=feet,
        feature_defaults=feature_defaults,
    )
    if legacy_v6 or legacy_v7:
        _apply_robot_motion_profile(config)
    elif legacy_v3 or legacy_v4 or legacy_v5 or legacy_v8:
        # These schemas carried verified servo calibration but a pre-measured
        # (URDF tibia-frame) kinematic model: replace only the kinematics.
        _apply_robot_kinematics_profile(config)
    return config


def encode_robot_config(cfg: RobotConfig) -> bytes:
    """Serialize a ``RobotConfig`` to its wire payload (inverse of decode).

    Used to generate golden vectors and for round-trip tests; the firmware owns
    the on-target serializer.
    """
    out = bytearray()
    out += struct.pack("<H", cfg.schema_version)
    name = cfg.robot_name.encode("ascii")[: ROBOT_NAME_LEN - 1]
    out += name + b"\x00" * (ROBOT_NAME_LEN - len(name))
    out += struct.pack(
        "<HHH", cfg.links.coxa_cmm, cfg.links.femur_cmm, cfg.links.tibia_cmm
    )
    out += struct.pack(
        "<HhH",
        cfg.geometry.home_radius_cmm,
        cfg.geometry.home_foot_z_cmm,
        cfg.geometry.coxa_lift_cmm,
    )
    for leg in cfg.legs:
        out += struct.pack(
            "<hhhh",
            leg.mount_x_dmm,
            leg.mount_y_dmm,
            leg.mount_z_dmm,
            leg.mount_yaw_cdeg,
        )
    for s in cfg.servos:
        out += struct.pack(
            "<BBBbhHH",
            s.id,
            s.leg,
            s.joint,
            s.sign,
            s.trim_ticks,
            s.min_tick,
            s.max_tick,
        )
    g = cfg.gait
    out += struct.pack(
        "<HHHBBB",
        g.body_height_mm,
        g.stride_len_mm,
        g.step_height_mm,
        g.duty_x255,
        g.speed_x255,
        g.gait,
    )
    if len(cfg.rc_input.channels) != NUM_RC_ANALOG_INPUTS:
        raise ValueError(f"expected {NUM_RC_ANALOG_INPUTS} RC calibration records")
    for channel in cfg.rc_input.channels:
        out += struct.pack(
            "<BBhhhBBBHH",
            channel.source,
            channel.type,
            channel.min_raw,
            channel.center_raw,
            channel.max_raw,
            channel.reversed,
            channel.deadband_x255,
            channel.expo_x255,
            channel.filter_tau_ms,
            channel.switch_debounce_ms,
        )
    body = cfg.body_command
    out += struct.pack(
        "<" + "H" * 14,
        body.max_forward_milli,
        body.max_reverse_milli,
        body.max_lateral_milli,
        body.max_yaw_milli,
        body.forward_accel_milli_per_s,
        body.forward_decel_milli_per_s,
        body.lateral_accel_milli_per_s,
        body.lateral_decel_milli_per_s,
        body.yaw_accel_milli_per_s,
        body.yaw_decel_milli_per_s,
        body.height_rise_mm_per_s,
        body.height_lower_mm_per_s,
        body.pose_translation_rate_mm_per_s,
        body.pose_rotation_rate_millirad_per_s,
    )
    for foot in cfg.feet:
        out += struct.pack(
            "<iHHHB",
            foot.pressure_baseline,
            foot.near_thresh,
            foot.touch_thresh,
            foot.load_thresh,
            foot.enabled,
        )
    out += struct.pack("<I", cfg.feature_defaults)
    return bytes(out)


# Per-leg coxa mount placement seeds (mirror config_schema.cpp; measured CAD,
# coxa rotation centres on the body mid-plane so mount z = 0).
_LEG_SEEDS = (
    (-656, -1156, 0, 13500),
    (656, -1156, 0, -13500),
    (698, 0, 0, -9000),
    (656, 1156, 0, -4500),
    (-656, 1156, 0, 4500),
    (-698, 0, 0, 9000),
)

def _apply_robot_kinematics_profile(cfg: RobotConfig) -> None:
    """Measured CAD kinematics (dimensions.md), mirroring config_schema.cpp."""
    cfg.links = LinkLengths(coxa_cmm=5200, femur_cmm=6651, tibia_cmm=11716)
    cfg.geometry = BodyGeometry(
        home_radius_cmm=12675, home_foot_z_cmm=-13173, coxa_lift_cmm=0
    )
    cfg.legs = [LegGeometry(*seed) for seed in _LEG_SEEDS]
    cfg.gait = GaitDefaults(
        body_height_mm=132,
        stride_len_mm=60,
        step_height_mm=30,
        duty_x255=159,
        speed_x255=128,
        gait=0,
    )

def _apply_robot_motion_profile(cfg: RobotConfig) -> None:
    _apply_robot_kinematics_profile(cfg)
    cfg.servos = []
    for leg in range(NUM_LEGS):
        for joint in range(JOINTS_PER_LEG):
            cfg.servos.append(
                ServoConfig(
                    id=leg * JOINTS_PER_LEG + joint + 1,
                    leg=leg,
                    joint=joint,
                    sign=1,
                    trim_ticks=0,
                    min_tick=1024,
                    max_tick=3072,
                )
            )
def default_robot_config() -> RobotConfig:
    """Compiled safe defaults, mirroring the HexNav CAD profile."""
    cfg = RobotConfig(schema_version=SCHEMA_VERSION, robot_name="HexNav")
    _apply_robot_motion_profile(cfg)
    cfg.rc_input = default_rc_input_calibration()
    cfg.body_command = BodyCommandLimits()
    cfg.feet = [FootSensorCal() for _ in range(NUM_FOOT_SENSORS)]
    # Only sensor polling defaults on so present boards stream raw data; all
    # richer/safety features stay off until requested (mirrors the firmware
    # kFeatureDefaultMask / protocol kFeatureDefaultEnabled baseline).
    cfg.feature_defaults = FEAT_SENSOR_POLLING
    return cfg


def default_rc_input_calibration() -> RcInputCalibration:
    """Safe logical input defaults that reproduce the firmware bridge ranges."""
    channels: list[RcChannelCalibration] = []
    for index in range(NUM_RC_ANALOG_INPUTS):
        if index < 4:
            channel_type = RC_CENTERED_ANALOG
            minimum, center, maximum = -1000, 0, 1000
            deadband = 13
            filter_tau_ms = 60
        elif index < 6:
            channel_type = RC_UNIPOLAR_ANALOG
            minimum, center, maximum = 0, 0, 1000
            deadband = 0
            filter_tau_ms = 120
        else:
            channel_type = RC_RELATIVE_ENCODER
            minimum, center, maximum = 0, 0, 2047
            deadband = 0
            filter_tau_ms = 0
        channels.append(
            RcChannelCalibration(
                source=index + 1,
                type=channel_type,
                min_raw=minimum,
                center_raw=center,
                max_raw=maximum,
                deadband_x255=deadband,
                filter_tau_ms=filter_tau_ms,
            )
        )
    return RcInputCalibration(channels)


# --------------------------------------------------------------------------- #
# RobotConfig validation (mirrors firmware validateRobotConfig).
# --------------------------------------------------------------------------- #
def validate_robot_config(config: RobotConfig) -> list[str]:
    """Return human-readable safety violations for a staged robot config.

    The firmware remains the final authority when ``CFG_VALIDATE`` runs, but
    calibration tools need deterministic local feedback before transmitting a
    multi-frame config update. This mirrors the firmware's structural and
    safety checks, adds fixed-width representation checks, and verifies that
    the configured neutral stance is geometrically reachable.
    """
    errors: list[str] = []

    if config.schema_version != SCHEMA_VERSION:
        errors.append(
            f"schema version {config.schema_version} is not {SCHEMA_VERSION}"
        )
    if len(config.robot_name.encode("ascii", errors="ignore")) >= ROBOT_NAME_LEN:
        errors.append(f"robot name must fit within {ROBOT_NAME_LEN - 1} ASCII bytes")

    for name, value in (
        ("coxa link", config.links.coxa_cmm),
        ("femur link", config.links.femur_cmm),
        ("tibia link", config.links.tibia_cmm),
    ):
        if not 1 <= value <= 0xFFFF:
            errors.append(f"{name} must be within 1..65535 centi-mm")

    geometry = config.geometry
    if not 1 <= geometry.home_radius_cmm <= 0xFFFF:
        errors.append("home radius must be within 1..65535 centi-mm")
    if not -0x8000 <= geometry.home_foot_z_cmm <= 0x7FFF:
        errors.append("home foot z is outside the signed centi-mm range")
    if not 0 <= geometry.coxa_lift_cmm <= 0xFFFF:
        errors.append("coxa lift is outside the unsigned centi-mm range")
    if all(
        0 < value <= 0xFFFF
        for value in (
            config.links.coxa_cmm,
            config.links.femur_cmm,
            config.links.tibia_cmm,
            geometry.home_radius_cmm,
        )
    ):
        coxa_mm = config.links.coxa_cmm / 100.0
        femur_mm = config.links.femur_cmm / 100.0
        tibia_mm = config.links.tibia_cmm / 100.0
        home_radius_mm = geometry.home_radius_cmm / 100.0
        home_z_mm = geometry.home_foot_z_cmm / 100.0
        home_distance_mm = math.hypot(home_radius_mm - coxa_mm, home_z_mm)
        if not abs(femur_mm - tibia_mm) <= home_distance_mm <= femur_mm + tibia_mm:
            errors.append("home stance is outside the two-link reach envelope")

    if len(config.legs) != NUM_LEGS:
        errors.append(f"expected {NUM_LEGS} leg geometry records")
    for index, leg in enumerate(config.legs):
        for name, value in (
            ("mount x", leg.mount_x_dmm),
            ("mount y", leg.mount_y_dmm),
            ("mount z", leg.mount_z_dmm),
            ("mount yaw", leg.mount_yaw_cdeg),
        ):
            if not -0x8000 <= value <= 0x7FFF:
                errors.append(f"leg {index} {name} is outside the signed range")

    gait = config.gait
    if gait.gait not in GAIT_NAMES:
        errors.append(f"gait {gait.gait} is unknown")
    if not 40 <= gait.body_height_mm <= 160:
        errors.append("body height must be within 40..160 mm")
    if not 0 <= gait.stride_len_mm <= 80:
        errors.append("stride length must be within 0..80 mm")
    if not 0 <= gait.step_height_mm <= 50:
        errors.append("step height must be within 0..50 mm")
    if not 0 <= config.feature_defaults <= 0xFFFFFFFF:
        errors.append("feature defaults are outside the unsigned 32-bit range")
    elif config.feature_defaults & ~(
        FEAT_FOOT_CONTACT
        | FEAT_TERRAIN_LEVELING
        | FEAT_SENSOR_POLLING
        | FEAT_PASSIVE_POSE_STREAM
        | FEAT_JETSON_CONTROL
    ):
        errors.append("feature defaults include unknown bits")

    rc_input = config.rc_input
    if len(rc_input.channels) != NUM_RC_ANALOG_INPUTS:
        errors.append(f"expected {NUM_RC_ANALOG_INPUTS} RC calibration records")
    sources: set[int] = set()
    for index, channel in enumerate(rc_input.channels):
        source_index = channel.source - 1
        if not 1 <= channel.source <= NUM_RC_ANALOG_INPUTS:
            errors.append(f"RC channel {index} source is outside 1..{NUM_RC_ANALOG_INPUTS}")
        elif channel.source in sources:
            errors.append(f"RC source {channel.source} is duplicated")
        sources.add(channel.source)
        expected_type = (
            RC_CENTERED_ANALOG
            if source_index < 4
            else RC_UNIPOLAR_ANALOG
            if source_index < 6
            else RC_RELATIVE_ENCODER
        )
        if channel.type != expected_type:
            errors.append(f"RC channel {index} has an incompatible type")
        if channel.min_raw >= channel.max_raw:
            errors.append(f"RC channel {index} minimum must be below maximum")
        if not channel.min_raw <= channel.center_raw <= channel.max_raw:
            errors.append(f"RC channel {index} center is outside its range")
        if channel.type == RC_CENTERED_ANALOG and channel.center_raw in (
            channel.min_raw,
            channel.max_raw,
        ):
            errors.append(f"RC channel {index} centered range is degenerate")
        if channel.reversed not in (0, 1):
            errors.append(f"RC channel {index} reverse must be 0 or 1")
        if not 0 <= channel.deadband_x255 <= 128:
            errors.append(f"RC channel {index} deadband exceeds the safe range")
        if not 0 <= channel.expo_x255 <= 255:
            errors.append(f"RC channel {index} expo is outside 0..255")
        if not 0 <= channel.filter_tau_ms <= RC_FILTER_TAU_MAX_MS:
            errors.append(f"RC channel {index} filter tau exceeds the safe range")
        if not 0 <= channel.switch_debounce_ms <= RC_SWITCH_DEBOUNCE_MAX_MS:
            errors.append(
                f"RC channel {index} switch debounce exceeds the safe range"
            )
    if sources != set(range(1, NUM_RC_ANALOG_INPUTS + 1)):
        errors.append("RC calibration must cover each logical source exactly once")

    body = config.body_command
    for name, value in (
        ("max forward", body.max_forward_milli),
        ("max reverse", body.max_reverse_milli),
        ("max lateral", body.max_lateral_milli),
        ("max yaw", body.max_yaw_milli),
    ):
        if not 1 <= value <= BODY_COMMAND_MAX_SCALE_MILLI:
            errors.append(f"{name} must be within 1..{BODY_COMMAND_MAX_SCALE_MILLI}")
    for name, value in (
        ("forward acceleration", body.forward_accel_milli_per_s),
        ("forward deceleration", body.forward_decel_milli_per_s),
        ("lateral acceleration", body.lateral_accel_milli_per_s),
        ("lateral deceleration", body.lateral_decel_milli_per_s),
        ("yaw acceleration", body.yaw_accel_milli_per_s),
        ("yaw deceleration", body.yaw_decel_milli_per_s),
    ):
        if not 1 <= value <= BODY_COMMAND_MAX_ACCEL_MILLI_PER_S:
            errors.append(
                f"{name} must be within 1..{BODY_COMMAND_MAX_ACCEL_MILLI_PER_S}"
            )
    if not 1 <= body.height_rise_mm_per_s <= BODY_HEIGHT_RATE_MAX_MM_PER_S:
        errors.append("height rise rate is outside the safe range")
    if not 1 <= body.height_lower_mm_per_s <= BODY_HEIGHT_RATE_MAX_MM_PER_S:
        errors.append("height lower rate is outside the safe range")
    if not 1 <= body.pose_translation_rate_mm_per_s <= BODY_POSE_TRANSLATION_RATE_MAX_MM_PER_S:
        errors.append("pose translation rate is outside the safe range")
    if not 1 <= body.pose_rotation_rate_millirad_per_s <= BODY_POSE_ROTATION_RATE_MAX_MILLIRAD_PER_S:
        errors.append("pose rotation rate is outside the safe range")

    if len(config.servos) != NUM_SERVOS:
        errors.append(f"expected {NUM_SERVOS} servo calibration records")
    servo_ids: set[int] = set()
    joint_slots: set[tuple[int, int]] = set()
    for index, servo in enumerate(config.servos):
        if not 1 <= servo.id <= 253:
            errors.append(f"servo {index} id must be within 1..253")
        elif servo.id in servo_ids:
            errors.append(f"servo id {servo.id} is duplicated")
        servo_ids.add(servo.id)
        if not 0 <= servo.leg < NUM_LEGS:
            errors.append(f"servo {index} leg must be within 0..{NUM_LEGS - 1}")
        if not 0 <= servo.joint < JOINTS_PER_LEG:
            errors.append(
                f"servo {index} joint must be within 0..{JOINTS_PER_LEG - 1}"
            )
        slot = (servo.leg, servo.joint)
        if slot in joint_slots:
            errors.append(f"servo slot leg {servo.leg}, joint {servo.joint} is duplicated")
        joint_slots.add(slot)
        if servo.sign not in (-1, 1):
            errors.append(f"servo {index} sign must be +1 or -1")
        if not -0x8000 <= servo.trim_ticks <= 0x7FFF:
            errors.append(f"servo {index} trim is outside the signed tick range")
        if not 0 <= servo.min_tick <= SERVO_MAX_TICK:
            errors.append(f"servo {index} min tick is outside 0..{SERVO_MAX_TICK}")
        if not 0 <= servo.max_tick <= SERVO_MAX_TICK:
            errors.append(f"servo {index} max tick is outside 0..{SERVO_MAX_TICK}")
        if servo.min_tick >= servo.max_tick:
            errors.append(f"servo {index} min tick must be less than max tick")
    expected_slots = {
        (leg, joint) for leg in range(NUM_LEGS) for joint in range(JOINTS_PER_LEG)
    }
    if joint_slots != expected_slots:
        errors.append("servo map must cover every leg and joint exactly once")

    if len(config.feet) != NUM_FOOT_SENSORS:
        errors.append(f"expected {NUM_FOOT_SENSORS} foot sensor calibration records")
    for index, foot in enumerate(config.feet):
        if not -0x80000000 <= foot.pressure_baseline <= 0x7FFFFFFF:
            errors.append(f"foot {index} baseline is outside the signed 32-bit range")
        for name, value in (
            ("near threshold", foot.near_thresh),
            ("touch threshold", foot.touch_thresh),
            ("load threshold", foot.load_thresh),
        ):
            if not 0 <= value <= 0xFFFF:
                errors.append(f"foot {index} {name} is outside the unsigned range")
        if foot.enabled:
            if not foot.near_thresh or not foot.touch_thresh or not foot.load_thresh:
                errors.append(f"foot {index} enabled calibration needs nonzero thresholds")
            if foot.load_thresh < foot.touch_thresh:
                errors.append(f"foot {index} load threshold must be at least touch")

    return errors


# --------------------------------------------------------------------------- #
# CFG_GET_SUMMARY decode + CFG_GET_BLOCK reassembly.
# --------------------------------------------------------------------------- #
def decode_config_summary(payload: bytes) -> ConfigSummary:
    """Decode the CFG_GET_SUMMARY response.

    Layout (config_api.cpp kGetSummary): schema_version(u16), payload_size(u16),
    block_max(u16), flags(u8: bit0 persistent, bit1 staged valid),
    feature_defaults(u32), robot_name(16).
    """
    if len(payload) < 2 + 2 + 2 + 1 + 4 + ROBOT_NAME_LEN:
        raise ConfigDecodeError(f"config summary too short ({len(payload)} bytes)")
    schema_version, payload_size, block_max = struct.unpack_from("<HHH", payload, 0)
    flags = payload[6]
    feature_defaults = struct.unpack_from("<I", payload, 7)[0]
    robot_name = _trim_name(payload[11 : 11 + ROBOT_NAME_LEN])
    return ConfigSummary(
        schema_version=schema_version,
        payload_size=payload_size,
        block_max=block_max,
        persistent=bool(flags & 0x01),
        staged_valid=bool(flags & 0x02),
        feature_defaults=feature_defaults,
        robot_name=robot_name,
    )


def decode_config_block(payload: bytes) -> tuple[int, bytes]:
    """Decode a CFG_GET_BLOCK response into ``(offset, data)``.

    Layout (config_api.cpp kGetBlock): offset(u16), len(u16), data[len].
    """
    if len(payload) < 4:
        raise ConfigDecodeError("config block response too short")
    offset, length = struct.unpack_from("<HH", payload, 0)
    data = payload[4 : 4 + length]
    if len(data) != length:
        raise ConfigDecodeError(
            f"config block declares {length} bytes but carries {len(data)}"
        )
    return offset, data


class ConfigBlockAssembler:
    """Reassembles the full serialized config payload from CFG_GET_BLOCK windows.

    The host issues windowed reads (offset/len) and feeds each response here;
    once :attr:`complete` is true, :meth:`decode` returns the ``RobotConfig``.
    """

    def __init__(self, total_len: int = CONFIG_PAYLOAD_SIZE) -> None:
        self.total_len = total_len
        self._buf = bytearray(total_len)
        self._filled = bytearray(total_len)  # 1 where a byte has been written

    def add_block(self, offset: int, data: bytes) -> None:
        end = offset + len(data)
        if offset < 0 or end > self.total_len:
            raise ConfigDecodeError(
                f"block [{offset}, {end}) is outside the {self.total_len}-byte payload"
            )
        self._buf[offset:end] = data
        for i in range(offset, end):
            self._filled[i] = 1

    def add_block_response(self, payload: bytes) -> None:
        """Convenience: decode a CFG_GET_BLOCK response and store it."""
        offset, data = decode_config_block(payload)
        self.add_block(offset, data)

    @property
    def complete(self) -> bool:
        return all(self._filled)

    def payload(self) -> bytes:
        if not self.complete:
            raise ConfigDecodeError("config payload is not fully assembled yet")
        return bytes(self._buf)

    def decode(self) -> RobotConfig:
        return decode_robot_config(self.payload())


# --------------------------------------------------------------------------- #
# Servo map + tick<->angle helpers (mirror servo_map.cpp).
# --------------------------------------------------------------------------- #
def _lround(x: float) -> int:
    """Round half away from zero, matching C ``lroundf``."""
    return int(math.floor(x + 0.5)) if x >= 0.0 else int(math.ceil(x - 0.5))


@dataclass
class JointCommand:
    tick: int = SERVO_CENTER_TICK
    clamped_low: bool = False
    clamped_high: bool = False
    unmapped: bool = False


def tick_to_angle(servo: ServoConfig, tick: int) -> float:
    """Convert a present-position tick to a URDF-zero-relative angle (radians).

    Mirrors ``ServoMap::tickToAngle``: applies the per-servo sign and trim about
    the 2048 center, 4096 ticks/rev.
    """
    offset_ticks = float(int(tick) - SERVO_CENTER_TICK - int(servo.trim_ticks))
    deg = offset_ticks / TICKS_PER_DEG
    return float(servo.sign) * deg * DEG_TO_RAD


def angle_to_tick(servo: ServoConfig, angle_rad: float) -> JointCommand:
    """Convert a joint angle (radians) to a clamped goal tick with clamp flags.

    Mirrors ``ServoMap::angleToTick``: sign/trim about center, clamp to the
    configured [min_tick, max_tick] travel, then defensively to [0, 4095].
    """
    out = JointCommand()
    deg = angle_rad * RAD_TO_DEG
    offset = _lround(deg * TICKS_PER_DEG)
    raw = SERVO_CENTER_TICK + int(servo.trim_ticks) + int(servo.sign) * offset

    lo = int(servo.min_tick)
    hi = int(servo.max_tick)
    if raw < lo:
        raw = lo
        out.clamped_low = True
    if raw > hi:
        raw = hi
        out.clamped_high = True
    if raw < 0:
        raw = 0
        out.clamped_low = True
    if raw > SERVO_MAX_TICK:
        raw = SERVO_MAX_TICK
        out.clamped_high = True

    out.tick = raw
    return out


class ServoMap:
    """Host-side view of the config servo map (mirror dxl::ServoMap)."""

    def __init__(self, config: RobotConfig) -> None:
        self._cfg = config
        self._by_slot = {(s.leg, s.joint): s for s in config.servos}
        self._by_id = {s.id: s for s in config.servos}

    def servo_for(self, leg: int, joint: int) -> ServoConfig | None:
        return self._by_slot.get((leg, joint))

    def servo_for_id(self, servo_id: int) -> ServoConfig | None:
        return self._by_id.get(servo_id)

    def angle_to_tick(self, leg: int, joint: int, angle_rad: float) -> JointCommand:
        s = self.servo_for(leg, joint)
        if s is None:
            return JointCommand(unmapped=True)
        return angle_to_tick(s, angle_rad)

    def tick_to_angle(self, leg: int, joint: int, tick: int) -> float:
        s = self.servo_for(leg, joint)
        if s is None:
            return 0.0
        return tick_to_angle(s, tick)


def servo_status_to_joint_angles(config: RobotConfig, servo_status) -> list:
    """Host fallback: map raw ``servo_status`` ticks to mapped joint angles.

    Used when the firmware ``joint_state`` stream (eax.1) is unavailable. For
    each ``ServoStatus`` whose id is in the config servo map, this reproduces the
    same value the firmware would emit: clamp the present position to the device
    range, convert tick->angle with the servo's sign/trim, and express it in
    centidegrees. Returns a list of ``telemetry.JointAngle`` (servos not in the
    map are skipped), matching the ``joint_state`` decode shape.
    """
    from .telemetry import JointAngle  # local import to avoid a cycle

    smap = ServoMap(config)
    joints: list[JointAngle] = []
    for s in getattr(servo_status, "servos", []):
        servo = smap.servo_for_id(s.id)
        if servo is None:
            continue
        tick = max(0, min(SERVO_MAX_TICK, s.position))
        rad = tick_to_angle(servo, tick)
        centideg = _lround(rad * RAD_TO_DEG * 100.0)
        centideg = max(-32768, min(32767, centideg))
        joints.append(JointAngle(servo.leg, servo.joint, centideg))
    return joints
