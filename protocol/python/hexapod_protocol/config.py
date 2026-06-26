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
ROBOT_NAME_LEN = 16  # incl. NUL terminator
SCHEMA_VERSION = 1

SERVO_CENTER_TICK = 2048
SERVO_MAX_TICK = 4095
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
    + NUM_LEGS * (4 * 2)  # legs (4 x int16 each)
    + NUM_SERVOS * (1 + 1 + 1 + 1 + 2 + 2 + 2)  # servos (10 bytes each)
    + (2 + 2 + 2 + 1 + 1 + 1)  # gait (9 bytes)
    + NUM_FOOT_SENSORS * (4 + 2 + 2 + 2 + 1)  # feet (11 bytes each)
    + 4  # feature_defaults
)  # == 331

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
    legs: list[LegGeometry] = field(default_factory=list)
    servos: list[ServoConfig] = field(default_factory=list)
    gait: GaitDefaults = field(default_factory=GaitDefaults)
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
    if len(payload) != CONFIG_PAYLOAD_SIZE:
        raise ConfigDecodeError(
            f"config payload is {len(payload)} bytes, expected {CONFIG_PAYLOAD_SIZE}"
        )

    o = 0
    schema_version = struct.unpack_from("<H", payload, o)[0]
    o += 2
    if schema_version != SCHEMA_VERSION:
        raise ConfigDecodeError(
            f"unsupported config schema version {schema_version} (expected {SCHEMA_VERSION})"
        )

    robot_name = _trim_name(payload[o : o + ROBOT_NAME_LEN])
    o += ROBOT_NAME_LEN

    links = LinkLengths(*struct.unpack_from("<HHH", payload, o))
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

    body_h, stride, step_h, duty, speed, gait = struct.unpack_from("<HHHBBB", payload, o)
    o += 9
    gait_defaults = GaitDefaults(body_h, stride, step_h, duty, speed, gait)

    feet: list[FootSensorCal] = []
    for _ in range(NUM_FOOT_SENSORS):
        baseline, near, touch, load, enabled = struct.unpack_from("<iHHHB", payload, o)
        feet.append(FootSensorCal(baseline, near, touch, load, enabled))
        o += 11

    feature_defaults = struct.unpack_from("<I", payload, o)[0]
    o += 4

    return RobotConfig(
        schema_version=schema_version,
        robot_name=robot_name,
        links=links,
        legs=legs,
        servos=servos,
        gait=gait_defaults,
        feet=feet,
        feature_defaults=feature_defaults,
    )


def encode_robot_config(cfg: RobotConfig) -> bytes:
    """Serialize a ``RobotConfig`` to its wire payload (inverse of decode).

    Used to generate golden vectors and for round-trip tests; the firmware owns
    the on-target serializer.
    """
    out = bytearray()
    out += struct.pack("<H", cfg.schema_version)
    name = cfg.robot_name.encode("ascii")[: ROBOT_NAME_LEN - 1]
    out += name + b"\x00" * (ROBOT_NAME_LEN - len(name))
    out += struct.pack("<HHH", cfg.links.coxa_cmm, cfg.links.femur_cmm, cfg.links.tibia_cmm)
    for leg in cfg.legs:
        out += struct.pack(
            "<hhhh", leg.mount_x_dmm, leg.mount_y_dmm, leg.mount_z_dmm, leg.mount_yaw_cdeg
        )
    for s in cfg.servos:
        out += struct.pack(
            "<BBBbhHH", s.id, s.leg, s.joint, s.sign, s.trim_ticks, s.min_tick, s.max_tick
        )
    g = cfg.gait
    out += struct.pack(
        "<HHHBBB", g.body_height_mm, g.stride_len_mm, g.step_height_mm, g.duty_x255, g.speed_x255, g.gait
    )
    for foot in cfg.feet:
        out += struct.pack(
            "<iHHHB", foot.pressure_baseline, foot.near_thresh, foot.touch_thresh, foot.load_thresh, foot.enabled
        )
    out += struct.pack("<I", cfg.feature_defaults)
    return bytes(out)


# Per-leg coxa mount placement seeds + sign rule (mirror config_schema.cpp).
_LEG_SEEDS = (
    (-656, -1156, -165, 13500),
    (656, -1156, -165, -13500),
    (698, 0, -165, -9000),
    (656, 1156, -165, -4500),
    (-656, 1156, -165, 4500),
    (-698, 0, -165, 9000),
)


def _is_left_leg(leg: int) -> bool:
    return leg in (0, 4, 5)


def default_robot_config() -> RobotConfig:
    """Compiled SAFE defaults, mirroring ``defaultRobotConfig`` (HexNav)."""
    cfg = RobotConfig(schema_version=SCHEMA_VERSION, robot_name="HexNav")
    cfg.links = LinkLengths(coxa_cmm=5608, femur_cmm=6651, tibia_cmm=2486)
    cfg.legs = [LegGeometry(*seed) for seed in _LEG_SEEDS]
    cfg.servos = []
    for i in range(NUM_SERVOS):
        leg = i // JOINTS_PER_LEG
        joint = i % JOINTS_PER_LEG
        cfg.servos.append(
            ServoConfig(
                id=joint * NUM_LEGS + leg + 1,
                leg=leg,
                joint=joint,
                sign=1 if _is_left_leg(leg) else -1,
                trim_ticks=0,
                min_tick=SERVO_CENTER_TICK - 1024,  # 1024
                max_tick=SERVO_CENTER_TICK + 1024,  # 3072
            )
        )
    cfg.gait = GaitDefaults(
        body_height_mm=40, stride_len_mm=60, step_height_mm=30, duty_x255=128, speed_x255=128, gait=0
    )
    cfg.feet = [FootSensorCal() for _ in range(NUM_FOOT_SENSORS)]
    cfg.feature_defaults = 0
    return cfg


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
