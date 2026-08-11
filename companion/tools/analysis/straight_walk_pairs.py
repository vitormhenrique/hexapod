#!/usr/bin/env python3
"""Fold goal + present waveforms per servo; report amplitude ratio and phase
lag, grouped as left/right mirror pairs (L1<->L2 rear, L4<->L5 front,
L3<->L6 mid). Asymmetric attenuation or lag => turning moment."""
import json
import math
import sys
from collections import defaultdict

PATH = sys.argv[1] if len(sys.argv) > 1 else "data/sessions/straight_walk.jsonl"
NBINS = 64
CENTER, TPD = 2048, 4096 / 360.0


def sid_from(leg, joint):
    return ((leg + 3) % 6) * 3 + joint + 1


goal_samples = defaultdict(list)
pres_samples = defaultdict(list)
for line in open(PATH):
    r = json.loads(line)
    d = r["data"]
    t = r["t"]
    if r["stream"] == "servo_goals" and d.get("goals"):
        for g in d["goals"]:
            tick = CENTER + g["angle_centideg"] / 100.0 * TPD
            goal_samples[sid_from(g["leg"], g["joint"])].append((t, tick))
    elif r["stream"] == "servo_status" and d.get("servos"):
        for s in d["servos"]:
            if s["position"] is not None and s["position"] >= 0:
                pres_samples[s["id"]].append((t, s["position"]))


def steady(v):
    n = len(v)
    return v[int(0.15 * n): int(0.85 * n)]


for k in goal_samples:
    goal_samples[k] = steady(goal_samples[k])
for k in pres_samples:
    pres_samples[k] = steady(pres_samples[k])
t0 = goal_samples[2][0][0]


def fold_score(samples, period):
    bins = [[] for _ in range(NBINS)]
    for t, v in samples:
        bins[min(NBINS - 1, int(((t - t0) % period) / period * NBINS))].append(v)
    gm = sum(v for _, v in samples) / len(samples)
    return sum(len(b) * (sum(b) / len(b) - gm) ** 2 for b in bins if b)


best_T, best_s = 0.640, -1
T = 0.640
for step_ms in (1.0, 0.1, 0.01):
    Tt = T - 5 * step_ms / 1000
    while Tt <= T + 5 * step_ms / 1000:
        s = fold_score(goal_samples[2], Tt)
        if s > best_s:
            best_s, best_T = s, Tt
        Tt += step_ms / 1000
    T = best_T


def wave(samples):
    bins = [[] for _ in range(NBINS)]
    for t, v in samples:
        bins[min(NBINS - 1, int(((t - t0) % T) / T * NBINS))].append(v)
    out = []
    for b in bins:
        out.append(sum(b) / len(b) if b else None)
    # fill gaps by neighbor interpolation
    for i in range(NBINS):
        if out[i] is None:
            j = (i + 1) % NBINS
            while out[j] is None:
                j = (j + 1) % NBINS
            k = (i - 1) % NBINS
            while out[k] is None:
                k = (k - 1) % NBINS
            out[i] = 0.5 * (out[j] + out[k])
    return out


def fundamental(w):
    """Return (amplitude, phase) of the first harmonic."""
    n = len(w)
    re = sum(w[i] * math.cos(2 * math.pi * i / n) for i in range(n)) * 2 / n
    im = sum(w[i] * math.sin(2 * math.pi * i / n) for i in range(n)) * 2 / n
    return math.hypot(re, im), math.atan2(im, re)


print(f"period {T*1000:.2f} ms")
print("servo (leg jnt)    amp_cmd amp_ach ratio   lag_deg  lag_ms")
rows = {}
for sid in range(1, 19):
    gw, pw = wave(goal_samples[sid]), wave(pres_samples[sid])
    ga, gp = fundamental(gw)
    pa, pp = fundamental(pw)
    dph = (pp - gp + math.pi) % (2 * math.pi) - math.pi
    lag_ms = dph / (2 * math.pi) * T * 1000
    rows[sid] = (ga, pa, pa / ga if ga > 1 else float("nan"), dph, lag_ms)

LEGN = ["L1 RL", "L2 RR", "L3 MR", "L4 FR", "L5 FL", "L6 ML"]
JN = ["coxa ", "femur", "tibia"]
for sid in range(1, 19):
    leg, j = (sid - 1) // 3, (sid - 1) % 3
    ga, pa, ratio, dph, lag = rows[sid]
    print(f"{sid:2d} ({LEGN[leg]} {JN[j]}):  {ga:6.1f}  {pa:6.1f}  {ratio:5.2f}"
          f"  {math.degrees(dph):+7.1f}  {lag:+6.1f}")

print("\nmirror pairs (ratio, lag_ms): left vs right")
pairs = [("rear ", 1, 2), ("mid  ", 6, 3), ("front", 5, 4)]
for name, lleg, rleg in pairs:
    for j in range(3):
        ls = (lleg - 1) * 3 + j + 1
        rs = (rleg - 1) * 3 + j + 1
        lr, ll = rows[ls][2], rows[ls][4]
        rr, rl = rows[rs][2], rows[rs][4]
        print(f"{name} {JN[j]}: L {lr:5.2f} {ll:+6.1f}ms   R {rr:5.2f} {rl:+6.1f}ms"
              f"   d_ratio={lr - rr:+.3f}  d_lag={ll - rl:+.1f}ms")
