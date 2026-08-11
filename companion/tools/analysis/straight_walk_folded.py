#!/usr/bin/env python3
"""Phase-folded straight-walk analysis.

Folds 60 s of ~9 Hz samples over the (precisely estimated) gait period to
reconstruct the average cycle waveform per servo for COMMANDED goals and
ACHIEVED present positions, then derives per-leg stance stroke vectors and
net body motion per cycle for both.
"""
import json
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/sessions/straight_walk.jsonl"
NBINS = 64

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
LEG_NAMES = ["L1 rear-left ", "L2 rear-right", "L3 mid-right ",
             "L4 front-right", "L5 front-left ", "L6 mid-left  "]


def solve_raw(x, z):
    pr = x - L1
    d = math.hypot(pr, z)
    cos_k = max(-1.0, min(1.0, (d * d - L2 * L2 - L3 * L3) / (2 * L2 * L3)))
    beta = -math.acos(cos_k)
    a = math.atan2(z, pr)
    b = math.atan2(L3 * math.sin(beta), L2 + L3 * math.cos(beta))
    return a - b, beta


FEMUR_REST, TIBIA_REST = solve_raw(HOME_RADIUS, HOME_FOOT_Z)


def foot_body(leg, coxa_a, femur_a, tibia_a):
    fr = femur_a + FEMUR_REST
    tr = tibia_a + TIBIA_REST
    pr = L2 * math.cos(fr) + L3 * math.cos(fr + tr)
    dz = L2 * math.sin(fr) + L3 * math.sin(fr + tr)
    horiz = pr + L1
    cx = horiz * math.cos(coxa_a)
    cy = horiz * math.sin(coxa_a)
    mx, my, mz, yaw = MOUNTS[leg]
    a = -(yaw + math.pi / 2.0)
    ca, sa = math.cos(a), math.sin(a)
    return (mx + ca * cx + sa * cy, my - sa * cx + ca * cy, dz + mz)


def servo_leg_joint(sid):
    slot = sid - 1
    return ((slot // 3) + 3) % 6, slot % 3


def sid_from(leg, joint):
    return ((leg + 3) % 6) * 3 + joint + 1


goal_samples = defaultdict(list)   # sid -> [(t, tick)]
pres_samples = defaultdict(list)   # sid -> [(t, tick)]
for line in open(PATH):
    r = json.loads(line)
    d = r["data"]
    t = r["t"]
    if r["stream"] == "servo_goals" and d.get("goals"):
        for g in d["goals"]:
            tick = CENTER + g["angle_centideg"] / 100.0 * TICKS_PER_DEG
            goal_samples[sid_from(g["leg"], g["joint"])].append((t, tick))
    elif r["stream"] == "servo_status" and d.get("servos"):
        for s in d["servos"]:
            if s["position"] is not None and s["position"] >= 0:
                pres_samples[s["id"]].append((t, s["position"]))

# Trim to the steady middle (drop first/last 15%).
def steady(v):
    n = len(v)
    return v[int(0.15 * n): int(0.85 * n)]


for k in goal_samples:
    goal_samples[k] = steady(goal_samples[k])
for k in pres_samples:
    pres_samples[k] = steady(pres_samples[k])

t0 = goal_samples[2][0][0]

# --- Estimate the exact gait period from servo 2's goal waveform -----------
def fold_score(samples, period):
    bins = [[] for _ in range(NBINS)]
    for t, v in samples:
        ph = ((t - t0) % period) / period
        bins[min(NBINS - 1, int(ph * NBINS))].append(v)
    gm = sum(v for _, v in samples) / len(samples)
    score = 0.0
    for b in bins:
        if b:
            mu = sum(b) / len(b)
            score += len(b) * (mu - gm) ** 2
    return score


best_T, best_s = None, -1
T = 0.640
for step_ms in (1.0, 0.1, 0.01):
    lo, hi = T - 5 * step_ms / 1000, T + 5 * step_ms / 1000
    n = 0
    Tt = lo
    while Tt <= hi:
        s = fold_score(goal_samples[2], Tt)
        if s > best_s:
            best_s, best_T = s, Tt
        Tt += step_ms / 1000
        n += 1
    T = best_T
print(f"estimated gait period: {T*1000:.2f} ms")


def folded_waveform(samples):
    bins = [[] for _ in range(NBINS)]
    for t, v in samples:
        ph = ((t - t0) % T) / T
        bins[min(NBINS - 1, int(ph * NBINS))].append(v)
    return [sum(b) / len(b) if b else None for b in bins]


# Waveforms per servo.
gwave = {sid: folded_waveform(goal_samples[sid]) for sid in range(1, 19)}
pwave = {sid: folded_waveform(pres_samples[sid]) for sid in range(1, 19)}

# Foot paths per leg over the folded cycle.
def foot_paths(waves):
    paths = {}
    for leg in range(6):
        pts = []
        for b in range(NBINS):
            a = []
            for joint in range(3):
                v = waves[sid_from(leg, joint)][b]
                if v is None:
                    a = None
                    break
                a.append(math.radians((v - CENTER) / TICKS_PER_DEG))
            pts.append(foot_body(leg, *a) if a else None)
        paths[leg] = pts
    return paths


for label, waves in (("COMMANDED", gwave), ("ACHIEVED", pwave)):
    paths = foot_paths(waves)
    print(f"\n=== {label} (folded average cycle) ===")
    print("leg              stroke dx     dy      |stroke|  skew(deg)  lift")
    total_dx = total_dy = 0.0
    for leg in range(6):
        pts = [p for p in paths[leg] if p]
        zmin = min(p[2] for p in pts)
        stance = [(i, p) for i, p in enumerate(paths[leg]) if p and p[2] < zmin + 4.0]
        # stroke: from max-y to min-y stance point (forward walk: foot travels -y)
        start = max(stance, key=lambda ip: ip[1][1])[1]
        end = min(stance, key=lambda ip: ip[1][1])[1]
        dx, dy = end[0] - start[0], end[1] - start[1]
        lift = max(p[2] for p in pts) - zmin
        skew = math.degrees(math.atan2(dx, -dy))
        total_dx += dx
        total_dy += dy
        print(f"{LEG_NAMES[leg]}  {dx:+7.2f} {dy:+7.2f}   {math.hypot(dx,dy):7.2f}"
              f"   {skew:+7.2f}   {lift:5.1f}")
    print(f"  net stance-stroke sum: dx={total_dx:+.2f} dy={total_dy:+.2f} "
          f"(dx>0 = feet skew right = body drifts LEFT)")
