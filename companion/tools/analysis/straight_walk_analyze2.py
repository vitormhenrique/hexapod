#!/usr/bin/env python3
"""Straight-walk drift, phase 2: use commanded stance masks (time-matched) to
select stance legs, then (a) rigid-fit achieved odometry and (b) per-leg
stance velocity vectors from present positions."""
import json
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/sessions/straight_walk.jsonl"

L1, L2, L3 = 52.00, 66.51, 117.16
HOME_RADIUS, HOME_FOOT_Z = 126.75, -131.73
TICKS_PER_DEG = 4096.0 / 360.0
CENTER = 2048
MOUNTS = [
    (-65.6, -115.6, 0.0, math.radians(135.0)),
    (65.6, -115.6, 0.0, math.radians(-135.0)),
    (69.8, 0.0, 0.0, math.radians(-90.0)),
    (65.6, 115.6, 0.0, math.radians(-45.0)),
    (-65.6, 115.6, 0.0, math.radians(45.0)),
    (-69.8, 0.0, 0.0, math.radians(90.0)),
]
LEG_NAMES = ["L1 rear-left", "L2 rear-right", "L3 mid-right",
             "L4 front-right", "L5 front-left", "L6 mid-left"]


def solve_raw(x, y, z):
    coxa = math.atan2(y, x)
    pr = math.hypot(x, y) - L1
    d = math.hypot(pr, z)
    cos_k = max(-1.0, min(1.0, (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
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


def foot_body(leg, coxa_a, femur_a, tibia_a):
    cx, cy, cz = forward_raw(coxa_a, femur_a + FEMUR_REST, tibia_a + TIBIA_REST)
    mx, my, mz, yaw = MOUNTS[leg]
    a = -(yaw + math.pi / 2.0)
    ca, sa = math.cos(a), math.sin(a)
    return (mx + ca * cx + sa * cy, my - sa * cx + ca * cy, cz + mz)


def servo_leg_joint(sid):
    slot = sid - 1
    return ((slot // 3) + 3) % 6, slot % 3


goal_frames = []    # (t, {leg: (x,y,z)})
status_frames = []  # (t, {leg: (x,y,z)})
for line in open(PATH):
    r = json.loads(line)
    d = r["data"]
    t = r["t"]
    if r["stream"] == "servo_goals" and d.get("goals"):
        angles = defaultdict(dict)
        for g in d["goals"]:
            angles[g["leg"]][g["joint"]] = math.radians(g["angle_centideg"] / 100.0)
        f = {leg: foot_body(leg, js[0], js[1], js[2])
             for leg, js in angles.items() if len(js) == 3}
        if len(f) == 6:
            goal_frames.append((t, f))
    elif r["stream"] == "servo_status" and d.get("servos"):
        angles = defaultdict(dict)
        for s in d["servos"]:
            pos = s.get("position")
            if pos is None or pos < 0:
                continue
            leg, joint = servo_leg_joint(s["id"])
            angles[leg][joint] = math.radians((pos - CENTER) / TICKS_PER_DEG)
        f = {leg: foot_body(leg, js[0], js[1], js[2])
             for leg, js in angles.items() if len(js) == 3}
        if len(f) == 6:
            status_frames.append((t, f))


def stance_mask(goal_feet):
    zmin = min(p[2] for p in goal_feet.values())
    return {leg for leg, p in goal_feet.items() if p[2] < zmin + 3.0}


def nearest_goal(t):
    lo, hi = 0, len(goal_frames) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if goal_frames[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    return goal_frames[lo]


def analyze(frames, label):
    bx = by = byaw = 0.0
    per_leg = defaultdict(lambda: [0.0, 0.0, 0.0])  # sum dx, dy, n
    prev = None
    for t, f in frames:
        if prev is not None:
            tp, fp = prev
            gt, gf = nearest_goal(0.5 * (t + tp))
            if abs(gt - 0.5 * (t + tp)) > 0.2:
                prev = (t, f)
                continue
            mask = stance_mask(gf)
            px, py, dx, dy, legs = [], [], [], [], []
            for leg in mask:
                p, q = f.get(leg), fp.get(leg)
                if p is None or q is None:
                    continue
                px.append(0.5 * (p[0] + q[0]))
                py.append(0.5 * (p[1] + q[1]))
                dx.append(p[0] - q[0])
                dy.append(p[1] - q[1])
                legs.append(leg)
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
                for k, leg in enumerate(legs):
                    per_leg[leg][0] += dx[k]
                    per_leg[leg][1] += dy[k]
                    per_leg[leg][2] += 1
        prev = (t, f)
    print(f"\n{label}:")
    print(f"  forward={by:+8.1f} mm  lateral={bx:+7.1f} mm (neg=left)  "
          f"yaw={math.degrees(byaw):+7.3f} deg")
    if abs(by) > 1:
        print(f"  drift {bx / (abs(by) / 1000.0):+6.1f} mm/m   "
              f"yaw {math.degrees(byaw) / (abs(by) / 1000.0):+6.3f} deg/m")
    print("  per-leg stance foot travel (body frame; straight walk => dx=0):")
    for leg in sorted(per_leg):
        sdx, sdy, n = per_leg[leg]
        if n:
            ang = math.degrees(math.atan2(sdx, -sdy))  # 0 = straight back
            print(f"    {LEG_NAMES[leg]:15s} dx={sdx:+8.1f} dy={sdy:+8.1f} mm "
                  f"(sum, n={int(n)})  skew={ang:+6.2f} deg")


print(f"goal frames: {len(goal_frames)}  status frames: {len(status_frames)}")
analyze(goal_frames, "COMMANDED (goals)")
analyze(status_frames, "ACHIEVED (present)")
