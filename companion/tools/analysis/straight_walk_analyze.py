#!/usr/bin/env python3
"""Analyze straight_walk.jsonl: rigid-fit body odometry from COMMANDED goals
and ACHIEVED present positions independently; report lateral drift and yaw."""
import json
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/sessions/straight_walk.jsonl"

# --- Robot model (matches persisted config, verified stock 2026-08-08) -----
L1, L2, L3 = 52.00, 66.51, 117.16
HOME_RADIUS, HOME_FOOT_Z = 126.75, -131.73
TICKS_PER_DEG = 4096.0 / 360.0
CENTER = 2048
MOUNTS = [  # (x_mm, y_mm, z_mm, yaw_rad) legs 0..5 (config order)
    (-65.6, -115.6, 0.0, math.radians(135.0)),
    (65.6, -115.6, 0.0, math.radians(-135.0)),
    (69.8, 0.0, 0.0, math.radians(-90.0)),
    (65.6, 115.6, 0.0, math.radians(-45.0)),
    (-65.6, 115.6, 0.0, math.radians(45.0)),
    (-69.8, 0.0, 0.0, math.radians(90.0)),
]


def solve_raw(x, y, z):
    coxa = math.atan2(y, x)
    horiz = math.hypot(x, y)
    pr = horiz - L1
    d = math.hypot(pr, z)
    cos_k = (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    cos_k = max(-1.0, min(1.0, cos_k))
    beta = -math.acos(cos_k)
    a = math.atan2(z, pr)
    b = math.atan2(L3 * math.sin(beta), L2 + L3 * math.cos(beta))
    return coxa, a - b, beta


FEMUR_REST, TIBIA_REST = solve_raw(HOME_RADIUS, 0.0, HOME_FOOT_Z)[1:3]


def forward_raw(coxa, femur_raw, tibia_raw):
    pr = L2 * math.cos(femur_raw) + L3 * math.cos(femur_raw + tibia_raw)
    dz = L2 * math.sin(femur_raw) + L3 * math.sin(femur_raw + tibia_raw)
    horiz = pr + L1
    return horiz * math.cos(coxa), horiz * math.sin(coxa), dz


def tick_to_angle(tick):
    # Stock servo map: sign +1, trim 0.
    return math.radians((tick - CENTER) / TICKS_PER_DEG)


def foot_body(leg, coxa_a, femur_a, tibia_a):
    cx, cy, cz = forward_raw(coxa_a, femur_a + FEMUR_REST, tibia_a + TIBIA_REST)
    mx, my, mz, yaw = MOUNTS[leg]
    a = -(yaw + math.pi / 2.0)
    ca, sa = math.cos(a), math.sin(a)
    bx = mx + ca * cx + sa * cy
    by = my - sa * cx + ca * cy
    return bx, by, cz + mz


def goal_servo_id(leg, joint):
    return ((leg + 3) % 6) * 3 + joint + 1


def servo_leg_joint(sid):
    slot = sid - 1
    cfg_leg, joint = slot // 3, slot % 3
    # invert v10 rotation: config leg -> logical leg
    return (cfg_leg + 3) % 6, joint


def rigid_odometry(frames):
    """frames: list of {leg: (x,y,z)}. Returns (fwd, lat, yaw) integrated.
    Stance feet are selected per frame: within 6 mm of that frame's deepest
    foot (works for commanded and tracked data alike)."""
    bx = by = byaw = 0.0
    prev = None
    for f in frames:
        if prev is not None:
            zmin_f = min(p[2] for p in f.values())
            zmin_q = min(p[2] for p in prev.values())
            px, py, dx, dy = [], [], [], []
            for leg, p in f.items():
                q = prev.get(leg)
                if q is None:
                    continue
                if p[2] > zmin_f + 6.0 or q[2] > zmin_q + 6.0:
                    continue
                px.append(0.5 * (p[0] + q[0]))
                py.append(0.5 * (p[1] + q[1]))
                dx.append(p[0] - q[0])
                dy.append(p[1] - q[1])
            n = len(px)
            if n >= 3:
                pbx, pby = sum(px) / n, sum(py) / n
                dbx, dby = sum(dx) / n, sum(dy) / n
                num = den = 0.0
                for k in range(n):
                    cx, cy = px[k] - pbx, py[k] - pby
                    ex, ey = dx[k] - dbx, dy[k] - dby
                    num += ex * cy - ey * cx
                    den += cx * cx + cy * cy
                dth = num / den if den > 0 else 0.0
                dcx = dth * pby - dbx
                dcy = -dth * pbx - dby
                c, s = math.cos(byaw), math.sin(byaw)
                bx += c * dcx - s * dcy
                by += s * dcx + c * dcy
                byaw += dth
        prev = f
    return bx, by, byaw


goal_frames = []
present_frames = []
for line in open(PATH):
    r = json.loads(line)
    d = r["data"]
    if r["stream"] == "servo_goals" and d.get("goals"):
        angles = defaultdict(dict)
        for g in d["goals"]:
            angles[g["leg"]][g["joint"]] = math.radians(g["angle_centideg"] / 100.0)
        f = {}
        for leg, js in angles.items():
            if len(js) == 3:
                f[leg] = foot_body(leg, js[0], js[1], js[2])
        if len(f) == 6:
            goal_frames.append(f)
    elif r["stream"] == "servo_status" and d.get("servos"):
        angles = defaultdict(dict)
        for s in d["servos"]:
            pos = s.get("position")
            if pos is None or pos < 0:
                continue
            leg, joint = servo_leg_joint(s["id"])
            angles[leg][joint] = tick_to_angle(pos)
        f = {}
        for leg, js in angles.items():
            if len(js) == 3:
                f[leg] = foot_body(leg, js[0], js[1], js[2])
        if len(f) == 6:
            present_frames.append(f)

print(f"goal frames: {len(goal_frames)}  present frames: {len(present_frames)}")
for name, frames in (("COMMANDED (goals)", goal_frames),
                     ("ACHIEVED (present)", present_frames)):
    if len(frames) < 10:
        print(f"{name}: not enough frames")
        continue
    fx, fy, yaw = rigid_odometry(frames)
    dist = math.hypot(fx, fy)
    print(f"\n{name}:")
    print(f"  forward={fy:+8.1f} mm   lateral={fx:+7.1f} mm (neg=left)")
    print(f"  net yaw={math.degrees(yaw):+7.3f} deg")
    if abs(fy) > 1:
        print(f"  lateral drift: {fx / (abs(fy) / 1000.0):+7.1f} mm per meter")
        print(f"  yaw rate: {math.degrees(yaw) / (abs(fy) / 1000.0):+7.3f} deg per meter")
